/*
 * ACS4 Flight Computer — Actuator Hub
 *
 * Aggregates commanded actuator state (4-aileron canard bank) into a
 * single timestamped snapshot shared between the producer (future
 * control-law, debug shell) and the consumer (ActuatorThread).
 *
 * Threading model mirrors SensorHub: all access goes through chSysLock
 * to keep struct updates atomic.
 *
 * Producers:
 *   - control-law / FSM: set_aileron_deg(), set_armed_request()
 *   - debug shell: same, plus set_sweep()
 *
 * Consumer (ActuatorThread):
 *   - snapshot() at 100 Hz, applies sweep (if active), pushes to ServoBankT75
 *   - feeds back via update_current_us() / update_armed()
 */

#pragma once

#include <array>
#include <cstdint>

namespace acs
{

constexpr uint8_t kAileronCount = 4;

struct AileronSweepState
{
    bool     active;         /* sweep enabled for this fin */
    float    min_deg;        /* sweep lower bound */
    float    max_deg;        /* sweep upper bound */
    uint32_t period_ms;      /* full triangle period */
    uint32_t start_time_ms;  /* chVTGetSystemTimeX at sweep start (ms) */
};

struct ActuatorSnapshot
{
    std::array<float, kAileronCount>    aileron_cmd_deg;     /* last commanded angle */
    std::array<uint16_t, kAileronCount> aileron_current_us;  /* actual PWM width */
    std::array<AileronSweepState, kAileronCount> sweep;
    uint32_t cmd_timestamp_us;
    bool     armed_request; /* what the producer wants */
    bool     armed;         /* what the hardware is currently doing */
};

class ActuatorHub
{
  public:
    ActuatorHub()                                = default;
    ~ActuatorHub()                               = default;
    ActuatorHub(const ActuatorHub &)             = delete;
    ActuatorHub &operator=(const ActuatorHub &)  = delete;
    ActuatorHub(ActuatorHub &&)                  = delete;
    ActuatorHub &operator=(ActuatorHub &&)       = delete;

    /* Producer side (control-law, shell) */

    void set_aileron_deg(uint8_t idx, float deg, uint32_t timestamp_us);
    void set_armed_request(bool armed);

    /**
     * @brief Enable a triangle-wave sweep between min_deg and max_deg on one
     *        fin. Cleared by set_aileron_deg() or set_sweep(idx, false, ...).
     *
     * @param start_time_ms  Result of chTimeI2MS(chVTGetSystemTimeX()) at
     *                       the moment the shell command was issued; used
     *                       as the phase-zero reference.
     */
    void set_sweep(uint8_t  idx,
                   bool     active,
                   float    min_deg,
                   float    max_deg,
                   uint32_t period_ms,
                   uint32_t start_time_ms);

    /* Consumer side (ActuatorThread) */

    void update_current_us(uint8_t idx, uint16_t us);
    void update_armed(bool armed);

    /**
     * @brief Atomic copy of the current actuator state.
     */
    ActuatorSnapshot snapshot();

  private:
    ActuatorSnapshot data_{};
};

/**
 * @brief Get reference to the global ActuatorHub singleton.
 */
ActuatorHub &actuator_hub();

}  // namespace acs
