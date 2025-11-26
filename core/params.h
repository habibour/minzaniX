/**
 * @file params.h
 * @brief Parameter storage and management interface
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize parameters to defaults
 */
void params_init(void);

/**
 * @brief Load parameters from persistent storage
 * @return true if successful
 */
bool params_load(void);

/**
 * @brief Save parameters to persistent storage
 * @return true if successful
 */
bool params_save(void);

/**
 * @brief Get PID parameters
 * @param params Output PID parameters
 */
void params_get_pid(pid_params_t *params);

/**
 * @brief Set PID parameters
 * @param params PID parameters
 */
void params_set_pid(const pid_params_t *params);

/**
 * @brief Get pointer to global PID parameters (for runtime tuning)
 * WARNING: Direct access - use with caution!
 * @return Pointer to global PID parameters
 */
pid_params_t* params_get_pid_ptr(void);

#ifdef __cplusplus
}
#endif
