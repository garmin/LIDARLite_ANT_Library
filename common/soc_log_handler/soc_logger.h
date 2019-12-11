/*
Copyright (c) 2019 Garmin Ltd. or its subsidiaries.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef SOC_LOG_H
#define SOC_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include "soc_log_defines.h"

// maximum size of buffer for log messages
#define MAX_SOC_LOG_BUFFER  256

/**@brief Function for logging from the library
 *
 * @param[in]   name    Name of the module
 * @param[in]   level   Corresponds directly to NRF LOG Levels
 *                      1, 2, 3, 4
 * @param[in]   msg     Ptr to buffer[], message to be logged
 */
void soc_log(
    char* name,
    uint32_t level,
    const char* msg,
    ...
    );


/**@brief Function for logging a hexdump from the library
 *        Not currently being used but is here for completeness
 *
 * @param[in]   name    Name of the module
 * @param[in]   level   Corresponds directly to NRF LOG Levels
 *                      1, 2, 3, 4
 * @param[in]   p_data  Data to be logged
 * @param[in]   len     length of data to log
 */
void soc_log_hexdump_module(
    char* name,
    uint32_t level,
    uint8_t p_data[],
    uint8_t len
    );

/**@brief   Setup the callback function access ptr
 *
 * @param[in]  ptr      Callback handler defined in main.c
 */
void soc_log_init (void (*ptr)(uint32_t, const char* string, ...));

/**@brief   Generic callback handler passed to library.  This must be
 *          have support code in the main.c application.
 *
 */
void log_handler(uint32_t LEVEL, const char* string, ...);

/**
 * Macros for ease of use. Do not directly override these.
 */

#define soc_log_error(NAME, ...) soc_log(NAME, SOC_LOG_LEVEL_ERROR, __VA_ARGS__)
#define soc_log_warning(NAME, ...) soc_log(NAME, SOC_LOG_LEVEL_WARN, __VA_ARGS__)
#define soc_log_info(NAME, ...) soc_log(NAME, SOC_LOG_LEVEL_INFO, __VA_ARGS__)
#define soc_log_debug(NAME, ...) soc_log(NAME, SOC_LOG_LEVEL_DEBUG, __VA_ARGS__)

#define soc_log_hexdump_error(NAME, p_data, len)
#define soc_log_hexdump_warn(NAME, p_data, len)
#define soc_log_hexdump_info(NAME, p_data, len)
#define soc_log_hexdump_debug(NAME, p_data, len)
#endif //SOC_LOG_H
