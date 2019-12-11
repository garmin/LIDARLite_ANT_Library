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

#ifndef __SOC_ERROR_HANDLER__
#define __SOC_ERROR_HANDLER__

// Ensure that this is in compiler space
#include "app_error.h"

#ifdef USE_SOC_LIBRARY_APP_ERROR

#include <stdbool.h>

/**
 * @brief SOC Library Error Handler
 *
 * Handle errors from a SOC Library
 *
 * @param[in]  error_code    The error code that occurred.
 * @param[in]  line_num      The line number that the error occurred on.
 * @param[in]  p_file_name   The event that was sent to the application from the SOC library.
 */
typedef void (*soc_lib_error_handler_t)(uint32_t error_code, uint32_t line_num, const uint8_t* p_file_name);


// clear out old macros for reuse
// these are defined in the app_error.h from the nrf5-sdk
#ifdef APP_ERROR_CHECK_BOOL
#undef APP_ERROR_CHECK_BOOL
#endif

#ifdef APP_ERROR_CHECK
#undef APP_ERROR_CHECK
#endif

#ifdef APP_ERROR_HANDLER
#undef APP_ERROR_HANDLER
#endif

void lib_error_handler_set(soc_lib_error_handler_t lib_error_handler);

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void lib_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);

/**@brief Macro for calling error handler function.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        lib_error_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)

/**@brief Macro for calling error handler function if supplied error code any other than NRF_SUCCESS.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
            APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)

/**@brief Macro for calling error handler function if supplied boolean value is false.
 *
 * @param[in] BOOLEAN_VALUE Boolean value to be evaluated.
 */
#define APP_ERROR_CHECK_BOOL(BOOLEAN_VALUE)                 \
    do                                                      \
    {                                                       \
        const bool LOCAL_BOOLEAN_VALUE = (BOOLEAN_VALUE);   \
        if (!LOCAL_BOOLEAN_VALUE)                           \
        {                                                   \
            APP_ERROR_HANDLER(0);                           \
        }                                                   \
    } while (0)

#endif  // USE_SOC_LIBRARY_APP_ERROR
#endif
