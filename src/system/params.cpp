/*
 * ACS4 Flight Computer — Runtime Parameter System Implementation
 */

#include "system/params.h"

#include <cstring>

extern "C" {
#include "ch.h"

#include <chprintf.h>
}

namespace acs
{

/* ── Parameter table ──────────────────────────────────────────────────── */
/* Add new parameters here. Keep alphabetically grouped by subsystem.     */

// clang-format off
static ParamEntry s_params[] = {
    /* ── Control: Roll ──────────────────────────────────────────────── */
    {"ctrl.kp_roll",            1.0f,   1.0f,   0.0f,   50.0f},
    {"ctrl.ki_roll",            0.0f,   0.0f,   0.0f,   10.0f},
    {"ctrl.kd_roll",            0.1f,   0.1f,   0.0f,   10.0f},

    /* ── Control: Pitch ─────────────────────────────────────────────── */
    {"ctrl.kp_pitch",           1.0f,   1.0f,   0.0f,   50.0f},
    {"ctrl.ki_pitch",           0.0f,   0.0f,   0.0f,   10.0f},
    {"ctrl.kd_pitch",           0.1f,   0.1f,   0.0f,   10.0f},

    /* ── Control: Yaw ───────────────────────────────────────────────── */
    {"ctrl.kp_yaw",             1.0f,   1.0f,   0.0f,   50.0f},
    {"ctrl.ki_yaw",             0.0f,   0.0f,   0.0f,   10.0f},
    {"ctrl.kd_yaw",             0.1f,   0.1f,   0.0f,   10.0f},

    /* ── Navigation / EKF ───────────────────────────────────────────── */
    {"nav.accel_noise",         0.5f,   0.5f,   0.001f, 10.0f},
    {"nav.gyro_noise",          0.01f,  0.01f,  0.0001f, 1.0f},

    /* ── FSM thresholds ─────────────────────────────────────────────── */
    {"fsm.liftoff_accel_g",     3.0f,   3.0f,   1.5f,   20.0f},
    {"fsm.liftoff_time_ms",     100.0f, 100.0f, 50.0f,  500.0f},
    {"fsm.apogee_vel_threshold", 5.0f,  5.0f,   1.0f,   50.0f},
};
// clang-format on

static constexpr int PARAM_COUNT =
    static_cast<int>(sizeof(s_params) / sizeof(s_params[0]));

/* ── Find by name ─────────────────────────────────────────────────────── */

static ParamEntry *find_param(const char *name)
{
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        if (strcmp(s_params[i].name, name) == 0)
        {
            return &s_params[i];
        }
    }
    return nullptr;
}

/* ── Public API ───────────────────────────────────────────────────────── */

bool param_get(const char *name, float &out)
{
    const ParamEntry *p = find_param(name);
    if (p == nullptr)
    {
        return false;
    }
    out = p->value;
    return true;
}

bool param_set(const char *name, float value)
{
    ParamEntry *p = find_param(name);
    if (p == nullptr)
    {
        return false;
    }
    if (value < p->min || value > p->max)
    {
        return false;
    }
    chSysLock();
    p->value = value;
    chSysUnlock();
    return true;
}

void param_reset_all()
{
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        s_params[i].value = s_params[i].default_val;
    }
}

void param_list(BaseSequentialStream *chp)
{
    chprintf(chp,
             "%-28s %12s %12s [%8s, %8s]\r\n",
             "Name",
             "Value",
             "Default",
             "Min",
             "Max");
    chprintf(chp,
             "-----------------------------------------------------------------"
             "---------\r\n");

    for (int i = 0; i < PARAM_COUNT; i++)
    {
        const auto &p = s_params[i];
        chprintf(chp,
                 "%-28s %12.4f %12.4f [%8.3f, %8.3f]\r\n",
                 p.name,
                 static_cast<double>(p.value),
                 static_cast<double>(p.default_val),
                 static_cast<double>(p.min),
                 static_cast<double>(p.max));
    }
}

ParamEntry *param_table(int &count)
{
    count = PARAM_COUNT;
    return s_params;
}

}  // namespace acs
