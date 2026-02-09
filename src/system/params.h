/*
 * ACS4 Flight Computer — Runtime Parameter System
 *
 * Static table of tunable parameters in RAM (defaults in Flash).
 * Changeable via shell without recompilation.
 */

#ifndef ACS4_PARAMS_H
#define ACS4_PARAMS_H

#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

/**
 * @brief A single runtime-tunable parameter.
 */
struct ParamEntry
{
    const char *name;
    float       value;
    float       default_val;
    float       min;
    float       max;
};

/**
 * @brief Get a parameter value by name.
 * @return true if found.
 */
bool param_get(const char *name, float &out);

/**
 * @brief Set a parameter value by name (clamped to [min, max]).
 * @return true if found and value was in range.
 */
bool param_set(const char *name, float value);

/**
 * @brief Reset all parameters to their defaults.
 */
void param_reset_all();

/**
 * @brief Print all parameters to a stream (shell `param list`).
 */
void param_list(BaseSequentialStream *chp);

/**
 * @brief Get direct access to the parameter table (for fast hot-path reads).
 * @param count [out] Number of entries.
 * @return Pointer to the param table.
 */
ParamEntry *param_table(int &count);

}  // namespace acs

#endif /* ACS4_PARAMS_H */
