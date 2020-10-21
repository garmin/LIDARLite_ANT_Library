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

#ifndef __LIDAR_LITE_DEFINES__
#define __LIDAR_LITE_DEFINES__

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_LITE_LIB_VERSION_LENGTH           11

/**
 * @brief Lidar Lite return type
 */
typedef enum
{
    LIDAR_LITE_SUCCESS,                         // The library command has started successfully
    LIDAR_LITE_ERROR_BUSY,                      // The library is currently busy and can not execute the command
    LIDAR_LITE_ERROR_INCORRECT_STATE,           // The command cannot be executed library's current state
    LIDAR_LITE_ERROR_INVALID_REGISTER,          // A write or read has occurred to on an invalid register address
    LIDAR_LITE_ERROR_INVALID_PARAM              // Error, value is out of range and can not be used
} lidar_lite_return_t;

/**
 * @brief Lidar Lite Configeurable Settings
 */
typedef enum
{
    LIDAR_LITE_MAX_ACQUISITION,
    LIDAR_LITE_QUICK_TERMINATION,
    LIDAR_LITE_DETECTION_SENSITIVITY,
} lidar_lite_config_t;

/**
 * @brief Lidar Lite Command Response Codes
 */
typedef enum
{
    LIDAR_LITE_COMMAND_SUCCESSFUL,
    LIDAR_LITE_ERROR_MEASUREMENT_FAILED_INVALID_PEAK,
    LIDAR_LITE_ERROR_CONFIGURATION_FAILED,
    LIDAR_LITE_ADC_PERIPHERAL_ERROR
} lidar_lite_evt_response_code_t;

/**
 * @brief Lidar Lite event callback function event definitions.
 */
typedef enum
{
    LIDAR_LITE_EVT_POWER_UP_COMPLETE,           // The FPGA has completed it's power up process and is ready to accept commands
    LIDAR_LITE_EVT_MEASUREMENT_COMPLETE,        // The FPGA has completed the measurement and is ready to accept commands
    LIDAR_LITE_EVT_READ_COMPLETE,               // Reading from the PFGA register has completed and is ready to accept commands
    LIDAR_LITE_EVT_TEMP_COMPLETE                // ADC has completed the temperature reading and computation
} lidar_lite_evt_type_t;


/**
 * @brief Lidar Lite FPGA register structure
 */
typedef struct
{
    uint8_t fpga_register_address;
    uint8_t fpga_register_value;
} lidar_lite_fpga_register_t;

/**
 * @brief Lidar Lite event structure
 */
typedef struct
{
    lidar_lite_evt_type_t type;                               ///< Event Type
    lidar_lite_evt_response_code_t response_code;             ///< Event Response Code
    union
    {
        uint16_t distance_cm;                                 ///< Data for @ref LIDAR_LITE_EVT_MEASUREMENT_DONE
        lidar_lite_fpga_register_t fpga_register;             ///< Data for @ref LIDAR_LITE_EVT_READ_COMPLETE
        int8_t board_temperature;                             ///< Data for @ref LIDAR_LITE_EVT_TEMP_COMPLETE
    } data;
}lidar_lite_evt_t;

/**
 * @brief Lidar Lite FPGA register structure
 */
typedef enum
{
    LIDAR_LITE_ASYNCHRONOUS_MODE,
    LIDAR_LITE_SYNCHRONOUS_MODE,
    LIDAR_LITE_ALWAYS_ON = 255
}lidar_lite_power_config_t;

/**
 * @brief Lidar Lite Quick Termination Modes
 */
typedef enum
{
    LIDAR_QUICK_TERMINATION_ENABLED = 0x00,
    LIDAR_QUICK_TERMINATION_DISABLED = 0x08
}lidar_lite_termination_config_t;

/**
 * @brief Lidar Lite event callback function type.
 *
 * @param[in] p_event Event information structure.
 */
typedef void (*lidar_lite_event_handler_t)(lidar_lite_evt_t * p_event);



#endif
