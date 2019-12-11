/*
Copyright (c) 2017 Garmin Ltd. or its subsidiaries.

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

#ifndef __APP_INTERFACE__
#define __APP_INTERFACE__

#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_UICR_CUST_ESN_OFFSET         1
#define SYSTEM_UICR_CUST_ANT_ID_OFFSET      4

/**
 * @brief Lidar Lite Configeurable Settings
 */
typedef enum
{
    NO_MEASUREMENT_IN_PROGRESS,
    MEASUREMENT_IN_PROGRESS,
} lidar_measurement_state_t;

 /** @brief  Update measurement state, measurement has been triggered
 *
 * @details  This function is called each time before a measurement is started on the LLv4
 *           This ensures that all interfaces (I2C, ANT, GPIO) can accurately represent the current state of the LLv4
 */
void measurement_started(void);

 /** @brief  Update measurement state, measurement has completed
 *
 * @details  This function is called each time a measurement has completed on the LLv4
 *           This ensures that all interfaces (I2C, ANT, GPIO) can accurately represent the current state of the LLv4
 */
void measurement_completed(void);

 /** @brief  Get measurement state
 *
 * @details  Get the current measurement State on the LLv4
 *           This is to be used by other application modules that require the current state of the lidar library
 *
 * @retval   NO_MEASUREMENT_IN_PROGRESS   The library is currently not taking a distance measurement
 * @retval   MEASUREMENT_IN_PROGRESS      The library is currently taking a distance measurement and connot should not be issued any commands until complete
 */
lidar_measurement_state_t get_measurement_state(void);

/**@brief Function for retrieving the ESN stored in the module at mfg.
 * @return 32-bit ESN
 */
uint32_t get_esn(void);

/**@brief Function for retrieving the ANT ID stored in the module at mfg.
 * @return 32-bit ANT ID
 */
uint32_t get_ant_id(void);

#endif
