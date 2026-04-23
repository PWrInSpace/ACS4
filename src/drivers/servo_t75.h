/*
 * ACS4 Flight Computer — SRT Servo T75 HV Servo-Bank Driver
 *
 * Drives 4 canard-fin ailerons from a single STM32 timer (TIM4 on the
 * production PCB) configured as 4 PWM channels at 330 Hz.
 *
 * Hardware mapping (production PCB, CUSTOM_H725 + CUSTOM_H725_JEDRZEJ):
 *   fin 1 → LINE_CH1_PWM → PD15 → TIM4_CH4 (ChibiOS channel index 3)
 *   fin 2 → LINE_CH2_PWM → PD14 → TIM4_CH3 (index 2)
 *   fin 3 → LINE_CH3_PWM → PD13 → TIM4_CH2 (index 1)
 *   fin 4 → LINE_CH4_PWM → PD12 → TIM4_CH1 (index 0)
 *
 * Control model:
 *   - Caller sets targets via set_angle_deg() or set_pulse_us().
 *   - tick(dt_ms) is called from ActuatorThread at 100 Hz; it applies
 *     a slew-rate limiter between current and target, then pushes the
 *     current value to the PWM channel.
 *
 * Fail-safe:
 *   - init() leaves the bank disarmed: channels are NOT enabled, pins stay LOW.
 *   - arm() enables all 4 channels at neutral (1500 µs by default).
 *   - disarm() disables all 4 channels: pins return to LOW (no pulse),
 *     servo receives no signal and either free-wheels or latches last position.
 *
 * Geometry decoupling:
 *   The driver does NOT know about body-frame axes. Fin indexing follows the
 *   PCB silkscreen (fin 1..4 = CH1..CH4_PWM). The future control-law mixer
 *   owns the fin-to-body-frame mapping.
 */

#pragma once

#include <cstdint>

#include "drivers/servo_t75_math.h"

/* Forward declaration keeps this header independent of HAL_USE_PWM being
 * enabled. On Nucleo (HAL_USE_PWM=FALSE) the class compiles fine; the
 * implementation in servo_t75.cpp stubs out the hardware-dependent body. */
struct PWMDriver;

namespace acs
{

/* =====================================================================
 * ServoT75Config — per-channel trim, limits, gain, slew rate
 * ===================================================================== */

struct ServoT75Config
{
    uint16_t neutral_us[4];     /* mechanical zero pulse per fin */
    uint16_t min_us[4];         /* hard lower bound per fin (safety stop) */
    uint16_t max_us[4];         /* hard upper bound per fin (safety stop) */
    int8_t   direction_sign[4]; /* +1 or -1, flipped per fin after bench calibration */

    float us_per_deg;     /* datasheet linear gain, 11.111 µs/deg */
    float max_angle_deg;  /* software deflection cap (±) before hardware clamp */
    float slew_us_per_ms; /* max rate of change of the pulse width */

    /**
     * @brief Defaults for the 4-canard rocket.
     *   - neutral 1500 µs (electrical center per datasheet)
     *   - hardware limits ±200 µs (≈ ±18°) — software loop is expected ±10°
     *   - direction +1 everywhere — flip per fin during bench calibration
     *   - slew 2 µs/ms (≈ 180°/s) — slower than the servo can mechanically
     *     move (≈ 500°/s) to avoid current spikes on the BEC rail
     *   - software angle clamp ±15° (cushion above ±10° control authority)
     */
    static constexpr ServoT75Config rocket_default()
    {
        return {
            .neutral_us     = {1500, 1500, 1500, 1500},
            .min_us         = {1300, 1300, 1300, 1300},
            .max_us         = {1700, 1700, 1700, 1700},
            .direction_sign = {+1, +1, +1, +1},
            .us_per_deg     = 11.111f,
            .max_angle_deg  = 15.0f,
            .slew_us_per_ms = 2.0f,
        };
    }
};

/* =====================================================================
 * ServoBankT75 — 4-channel PWM driver
 * ===================================================================== */

class ServoBankT75
{
  public:
    static constexpr uint8_t FIN_COUNT = 4;

    ServoBankT75()                                 = default;
    ~ServoBankT75()                                = default;
    ServoBankT75(const ServoBankT75 &)             = delete;
    ServoBankT75 &operator=(const ServoBankT75 &)  = delete;
    ServoBankT75(ServoBankT75 &&)                  = delete;
    ServoBankT75 &operator=(ServoBankT75 &&)       = delete;

    /**
     * @brief Initialize the timer in PWM mode. Bank starts disarmed.
     *
     * Calls pwmStart() on the supplied driver with a 330 Hz / 1 µs-tick
     * configuration. All channels remain disabled (pins LOW) until arm().
     *
     * @param pwm  Pointer to a ChibiOS PWMDriver (e.g. &PWMD4).
     * @param cfg  Servo-bank configuration.
     * @return true on success.
     */
    [[nodiscard]] bool init(PWMDriver *pwm, const ServoT75Config &cfg);

    /**
     * @brief Enable all 4 channels at neutral. Required before any output
     *        pulses appear on the pins.
     */
    void arm();

    /**
     * @brief Disable all 4 channels. Pins return to LOW, no pulse emitted.
     *        Safe-by-default state after init and between flights.
     */
    void disarm();

    /**
     * @brief Command a deflection angle (degrees) on one fin.
     *
     * Maps through angle_to_pulse_us(). Has no immediate effect on the pin —
     * the slew limiter in tick() moves current toward this new target.
     *
     * @param fin_idx  0..FIN_COUNT-1 (fin 1 = index 0).
     * @param deg      Commanded angle in degrees.
     */
    void set_angle_deg(uint8_t fin_idx, float deg);

    /**
     * @brief Bench override: command a raw pulse width on one fin.
     *
     * Bypasses the angle-to-µs conversion (still respects min_us/max_us).
     *
     * @param fin_idx  0..FIN_COUNT-1.
     * @param us       Commanded pulse width in µs.
     */
    void set_pulse_us(uint8_t fin_idx, uint16_t us);

    /**
     * @brief Advance the slew limiter by `dt_ms` milliseconds and push the
     *        current pulse width to the hardware (only while armed).
     *
     * Should be called at a steady rate from ActuatorThread (100 Hz →
     * dt_ms = 10).
     */
    void tick(uint32_t dt_ms);

    [[nodiscard]] bool is_initialized() const
    {
        return initialized_;
    }

    [[nodiscard]] bool is_armed() const
    {
        return armed_;
    }

    /**
     * @brief Current pulse width actually being output to the pin (µs).
     *        Returns 0 if fin_idx is out of range.
     */
    [[nodiscard]] uint16_t current_pulse_us(uint8_t fin_idx) const
    {
        return (fin_idx < FIN_COUNT) ? current_us_[fin_idx] : 0;
    }

    /**
     * @brief Last commanded target pulse width before slew (µs).
     */
    [[nodiscard]] uint16_t target_pulse_us(uint8_t fin_idx) const
    {
        return (fin_idx < FIN_COUNT) ? target_us_[fin_idx] : 0;
    }

    /**
     * @brief Timer channel index used by a given fin (for debugging).
     */
    [[nodiscard]] static uint8_t timer_channel_for_fin(uint8_t fin_idx);

  private:
    servo_t75::Limits limits_for(uint8_t fin_idx) const;

    PWMDriver     *pwm_        = nullptr;
    ServoT75Config cfg_        = {};
    bool           initialized_ = false;
    bool           armed_       = false;

    uint16_t target_us_[FIN_COUNT]  = {0, 0, 0, 0};
    uint16_t current_us_[FIN_COUNT] = {0, 0, 0, 0};
};

/**
 * @brief Get pointer to the global servo-bank instance.
 * @return Pointer to initialized ServoBankT75, or nullptr if unavailable.
 */
ServoBankT75 *servo_bank_instance();

}  // namespace acs
