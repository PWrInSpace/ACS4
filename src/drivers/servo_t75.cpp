/*
 * ACS4 Flight Computer — SRT Servo T75 HV Servo-Bank Driver Implementation
 */

#include "drivers/servo_t75.h"

#include "drivers/servo_t75_math.h"

extern "C" {
#include "hal.h"
}

namespace acs
{

/* =====================================================================
 * Fin → TIM4 channel index mapping
 *
 * Physical fin numbering follows the PCB silkscreen CH1..CH4_PWM:
 *   fin 1 → PD15 → TIM4_CH4 → ChibiOS channel index 3
 *   fin 2 → PD14 → TIM4_CH3 → index 2
 *   fin 3 → PD13 → TIM4_CH2 → index 1
 *   fin 4 → PD12 → TIM4_CH1 → index 0
 * ===================================================================== */

static constexpr uint8_t kFinToTimerChannel[ServoBankT75::FIN_COUNT] = {3, 2, 1, 0};

uint8_t ServoBankT75::timer_channel_for_fin(uint8_t fin_idx)
{
    return (fin_idx < FIN_COUNT) ? kFinToTimerChannel[fin_idx] : 0xFF;
}

#if HAL_USE_PWM == TRUE

/* =====================================================================
 * 330 Hz @ 1 MHz tick PWM configuration (TIM4, 4 channels)
 * ===================================================================== */

static const PWMConfig g_servo_pwm_cfg = {
    .frequency = 1'000'000U, /* 1 MHz tick → 1 µs per count */
    .period    = 3030U,       /* 3030 µs → 330.03 Hz */
    .callback  = nullptr,
    .channels  = {
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, /* CH1 = PD12 = fin 4 */
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, /* CH2 = PD13 = fin 3 */
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, /* CH3 = PD14 = fin 2 */
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, /* CH4 = PD15 = fin 1 */
    },
    .cr2  = 0,
    .bdtr = 0,
    .dier = 0,
};

/* =====================================================================
 * init / arm / disarm
 * ===================================================================== */

bool ServoBankT75::init(PWMDriver *pwm, const ServoT75Config &cfg)
{
    if (pwm == nullptr)
    {
        return false;
    }

    pwm_ = pwm;
    cfg_ = cfg;

    pwmStart(pwm_, &g_servo_pwm_cfg);

    for (uint8_t i = 0; i < FIN_COUNT; ++i)
    {
        target_us_[i]  = cfg_.neutral_us[i];
        current_us_[i] = cfg_.neutral_us[i];
    }

    armed_       = false;
    initialized_ = true;
    return true;
}

void ServoBankT75::arm()
{
    if (!initialized_)
    {
        return;
    }

    armed_ = true;

    for (uint8_t i = 0; i < FIN_COUNT; ++i)
    {
        target_us_[i]  = cfg_.neutral_us[i];
        current_us_[i] = cfg_.neutral_us[i];
        pwmEnableChannel(pwm_, kFinToTimerChannel[i], static_cast<pwmcnt_t>(current_us_[i]));
    }
}

void ServoBankT75::disarm()
{
    if (!initialized_)
    {
        return;
    }

    for (uint8_t i = 0; i < FIN_COUNT; ++i)
    {
        pwmDisableChannel(pwm_, kFinToTimerChannel[i]);
    }

    armed_ = false;
}

/* =====================================================================
 * set / tick
 * ===================================================================== */

servo_t75::Limits ServoBankT75::limits_for(uint8_t fin_idx) const
{
    return {
        .neutral_us     = cfg_.neutral_us[fin_idx],
        .min_us         = cfg_.min_us[fin_idx],
        .max_us         = cfg_.max_us[fin_idx],
        .direction_sign = cfg_.direction_sign[fin_idx],
        .us_per_deg     = cfg_.us_per_deg,
    };
}

void ServoBankT75::set_angle_deg(uint8_t fin_idx, float deg)
{
    if (!initialized_ || fin_idx >= FIN_COUNT)
    {
        return;
    }

    target_us_[fin_idx] =
        servo_t75::angle_to_pulse_us(deg, limits_for(fin_idx), cfg_.max_angle_deg);
}

void ServoBankT75::set_pulse_us(uint8_t fin_idx, uint16_t us)
{
    if (!initialized_ || fin_idx >= FIN_COUNT)
    {
        return;
    }

    if (us < cfg_.min_us[fin_idx])
    {
        us = cfg_.min_us[fin_idx];
    }
    else if (us > cfg_.max_us[fin_idx])
    {
        us = cfg_.max_us[fin_idx];
    }

    target_us_[fin_idx] = us;
}

void ServoBankT75::tick(uint32_t dt_ms)
{
    if (!initialized_)
    {
        return;
    }

    for (uint8_t i = 0; i < FIN_COUNT; ++i)
    {
        current_us_[i] =
            servo_t75::slew_step(current_us_[i], target_us_[i], cfg_.slew_us_per_ms, dt_ms);

        if (armed_)
        {
            pwmEnableChannel(
                pwm_, kFinToTimerChannel[i], static_cast<pwmcnt_t>(current_us_[i]));
        }
    }
}

#else /* HAL_USE_PWM == FALSE (Nucleo dev build — no PWM hardware) */

bool ServoBankT75::init(PWMDriver *, const ServoT75Config &)
{
    return false;
}

void ServoBankT75::arm()
{
}

void ServoBankT75::disarm()
{
}

void ServoBankT75::set_angle_deg(uint8_t, float)
{
}

void ServoBankT75::set_pulse_us(uint8_t, uint16_t)
{
}

void ServoBankT75::tick(uint32_t)
{
}

servo_t75::Limits ServoBankT75::limits_for(uint8_t) const
{
    return {};
}

#endif /* HAL_USE_PWM */

}  // namespace acs
