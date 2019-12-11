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

#ifndef __LIDAR_CONFIG_NVM__
#define __LIDAR_CONFIG_NVM__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_sdh_ant.h"
#include "fds.h"
#include "ant_message_parser.h"
#include "lidar_lite_defines.h"

/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0xBFFF)
#define CONFIG_REC_KEY  (0x7010)

/** Default Configuration States */
#define DEFAULT_SECONDARY_I2C_ADDR          0x00
#define DEFAULT_I2C_CONFIGURATION           0xF0                    // Default is ON
#define DEFAULT_POWER_MODE                  LIDAR_LITE_ALWAYS_ON    // Default
#define DEFAULT_MEASUREMENT_INTERVAL        0xFF                    // Measurement interval is OFF 0xFF
#define DEFAULT_DETECTION_SENSITIVITY       0x00
#define DEFAULT_ACQUISITION_COUNT           0xFF
#define DEFAULT_QUICK_TERMINATION           LIDAR_QUICK_TERMINATION_DISABLED
#define DEFAULT_HIGH_ACCURACY_COUNT         0x14                    // default is set to 20 samples for smoothing
#define DEFAULT_DISABLE_NVM_STORAGE         0x00                    // Off by default.
#define NVM_STORAGE_DISABLED                DEFAULT_DISABLE_NVM_STORAGE
#define NVM_STORAGE_ENABLED                 0x11

/** States Enum */
typedef enum {
    initializing = 0,
    initialized = 1,
    num_states
} lidar_config_nvm_state_t;

typedef struct
{
    uint8_t HEAD[3];    // Start block in RAM
    uint8_t enable_nvm_storage;
    uint8_t secondary_slave_address;
    uint8_t i2c_enabled;
    uint8_t power_mode;
    uint8_t measurement_interval;
    uint8_t detection_sensitivity;
    uint8_t acquisition_count;
    uint8_t quick_termination;
    uint8_t high_accuracy_count;
    uint8_t TAIL[3];    // End block in RAM
} lidar_fpga_config_t;

// Scheduler structures and enumerations for NVM
typedef enum
{
    NVM_GARBAGE_COLLECTION,
    NVM_INIT_COMPLETE,
    NVM_UPDATE_FAILURE,
    NVM_UNDEFINED
} ll_nvm_evt_type_t;

typedef struct
{
    ll_nvm_evt_type_t type;
    uint8_t response_code;
} ll_nvm_evt_t;


/**
 * @brief Lidar Lite NVM event callback function type.
 *
 * @param[in] p_event Event information structure.
 */
typedef void (*lidar_nvm_event_handler_t)(ll_nvm_evt_t * p_event);

// prototype definitions

/** @brief Lidar Lite NVM intialization routine
 *
 * @param[in] event_handler     Call back function to return status to main application.
 *
 * @retval  Return codes from fds.h file SDK `15.0
 *          FDS_SUCCESS = NRF_SUCCESS    operation was successful
 *          FDS_ERR_USER_LIMIT_REACHED   If the maximum number of registered callbacks is reached.
 *          FDS_ERR_NO_PAGES             If there is no space available in flash memory to install the
 *                                       file system.
 */
ret_code_t lidar_config_nvm_init(lidar_nvm_event_handler_t event_handler);

/**@brief   Function to read a record from NVM
 *
 * @details Function will read and populate the current FPGA record in NVM
 *
 * @param[in] lidar_fpga_config        Structure to be filled with data from NVM storage
 *
 * @retval  Return codes from fds.h file SDK `15.0
 *          FDS_SUCCESS                     // operation was successful
 *          FDS_ERR_NOT_INITIALIZED         // module is not initialized
 *          FDS_ERR_NULL_ARG                // Argument is NULL
 *          FDS_ERR_NOT_FOUND               // no matching record found
 *          FDS_ERR_CRC_CHECK_FAILED        // The CRC check for the record failed
 *          FDS_ERR_NO_OPEN_RECORDS         // the record is not openn
 */
ret_code_t lidar_config_nvm_read(lidar_fpga_config_t * lidar_fpga_config);

/**@brief   Function to update a record in the NVM with updated data
 *          Calls to fds module: fds_record_find() and fds_record_update()
 *
 * @details Function will write and store the current configuration information in a record in NVM
 *
 * @param[in] updated_config        Configuration data to be written to NVM
 *
 * @retval  Return codes from fds.h file SDK 15.0
 *          FDS_SUCCESS                     // operation was successful
 *          FDS_ERR_NOT_INITIALIZED         // the module is not initialized
 *          FDS_ERR_INVALID_ARG             // the fle ID or the record key is invalid
 *          FDS_ERR_NULL_ARG                // Arguments are NULL
 *          FDS_ERR_NOT_FOUND               // no matching record found
 *          FDS_ERR_UNALIGNED_ADDR          // record data no aligned to a 4 byte boundary
 *          FDS_ERR_RECORD_TOO_LARGE        // record data exceeds the maximum length
 *          FDS_ERR_NO_SPACE_IN_QUEUES      // operation queue is full or there are more record chunks than can be buffered
 *          FDS_ERR_NO_SPACE_IN_FLASH       // not enough free space in flash to store record
 */
ret_code_t lidar_config_nvm_update(lidar_fpga_config_t const * updated_config);

/**@brief   Function to get the secondary slave address from NVM
 *
 * @details Function will read and return the secondary slave address
 *
 * @param[out]  uint8_t     slave_address
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_config_get_slave_address(uint8_t *slave_address);

/**@brief   Function to set the secondary slave address in NVM
 *
 * @details Function will write the secondary slave address to NVM
 *
 * @param[in]  uint8_t     slave_address
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_config_set_slave_address(uint8_t slave_address);

/**@brief   Function to enable or disable TWIS from over ANT
 *
 * @param[in]  ant_i2c_modes_template_t enumerated types for I2C mode.
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_config_set_twis_mode(ant_i2c_modes_template_t mode);

/**@brief   Function to return the value of twis mode from NVM
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_config_get_twis_mode();

/**@brief   Function to return the value of the power mode setting
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_power_mode();

/**@brief   Function to set the power mode
 *
 * @param[in]  lidar_Lite_power_config_t  mode
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_power_mode(lidar_lite_power_config_t mode);

/**@brief   Function to return the value of the measurement interval
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_measurement_interval();

/**@brief   Function to set the measurement interval
 *
 * @param[in]  uint8_t  Interval
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_measurement_interval(uint8_t interval);

/**@brief   Function to set the power level and measurement interval, combined
 *
 * @param[in]  lidar_Lite_power_config_t    power_mode
 * @param[in]  uint8_t                      interval
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_power_and_interval(lidar_lite_power_config_t power_mode, uint8_t interval);

/**@brief   Function to restore the LLV4 to factory defaults
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_factory_reset();

/**@brief   Function to set the detection sensitivity
 *
 * @param[in]  uint8_t  sensitivity
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_detection_sensitivity(uint8_t sensitivity);

/**@brief   Function to return the detection sensitivity level
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_detection_sensitivity();

/**@brief   Function to return the acquisition count
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_acquisition_count();

/**@brief   Function to set the acquisition count
 *
 * @param[in]  uint8_t  count
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_acquisition_count(uint8_t count);

/**@brief   Function to return the quick termination mode
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_quick_termination_mode();

/**@brief   Function to set the quick termination mode
 *
 * @param[in]  lidar_lite_termination_config_t  mode
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_quick_termination_mode(lidar_lite_termination_config_t mode);

/**@brief   Function to set the high accuracy count
 *
 * @param[in] count : Value 0x00 through to 0xFF
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_high_accuracy_count(uint8_t count);

/**@brief   Function to return the high accuracy count value
 *
 * @retval  Value as stored in the NVM record
 */
uint8_t lidar_nvm_get_high_accuracy_count();

/**@brief   Function to return the Write access mode setting
 *
 * @retval  NVM enabled         returns true
 *          NVM NOT enabled     returns false
 */
bool lidar_nvm_get_access_mode();

/**@brief   Function to set the Write access mode
 *
 * @param[in]  uint8_t  mode
 *
 * @retval Return codes from fds.h file SDK 15.0, see above in lidar_config_nvm_update
 */
ret_code_t lidar_nvm_set_access_mode(uint8_t mode);
#endif
