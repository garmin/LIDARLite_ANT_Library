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

#ifndef __SERIAL_TWIS_PARSER__
#define __SERIAL_TWIS_PARSER__

#include <stdbool.h>
#include <stdint.h>

#define LL_MAX_REGISTER_SIZE_BYTES          2       // The maximum number of bytes that is returned from any read register request

// Lidar Light Registers
#define LL_REGISTER_ACQ_COMMAND             0x00    // Device Command
#define LL_REGISTER_STATUS                  0x01    // System Status
#define LL_REGISTER_ACQUISITION_COUNT       0x05    // Acquisition count setting
#define LL_REGISTER_FULL_DELAY_LOW          0x10    // Distance measurement low byte
#define LL_REGISTER_FULL_DELAY_HIGH         0x11    // Distance measurement high byte
#define LL_REGISTER_UNIT_ID_0               0x16    // Unique serial number, byte 0, least significant byte
#define LL_REGISTER_UNIT_ID_1               0x17    // Unique serial number, byte 1
#define LL_REGISTER_UNIT_ID_2               0x18    // Unique serial number, byte 2
#define LL_REGISTER_UNIT_ID_3               0x19    // Unique serial number, byte 3, Most significatn byte
#define LL_REGISTER_I2C_SEC_ADDR            0x1A    // Register read to return Secondary I2C address
#define LL_REGISTER_I2C_CONFIG              0x1B    // Allow for enable / disable of I2C addresses both default and secondary
#define LL_REGISTER_DETECTION_SENSITIVITY   0x1C    // Detection Sensitivity
#define LL_REGISTER_LIB_VERSION             0x30    // nRF Library Version
#define LL_REGISTER_CORR_DATA               0x52    // Two byte correlation record element
#define LL_REGISTER_GVER_LO                 0x72    // Garmin software version low byte
#define LL_REGISTER_GVER_HI                 0x73    // Garmin software version high byte

// Write commands for ACQ_COMMAND register
#define ACQ_COMMAND_RESET                   0x00    // Reset the FPGA
#define ACQ_COMMAND_TAKE_DISTANCE           0x03    // Take Distance without receiver bias correction
#define ACQ_COMMAND_TAKE_DISTANCE_BIAS      0x04    // Take Distance with receiver bias correction

#define LL_STATUS_BUSY                      0x01    // Used to indicate that the library is currently busy taking a measurement
#define LL_STATUS_INTERFACE_LOW_PWR         0x08    // Used to indicate that the library is currently in low power and I2C read/writes take longer

// definitions for the RADIO configuration
#define RADIO_ON                            0xFF    // On command recieved from I2C master
#define RADIO_OFF                           0x00    // Off command recieved from I2C master
#define RADIO_STATUS_NOT_AVAILABLE          0x55    // Status of Radio was not available error code

#define LL_TEMP_MEASUREMENT                 0xe0
#define LL_HARDWARE_VERSION                 0xe1
#define LL_POWER_MODE                       0xe2
#define LL_MEASUREMENT_INTERVAL             0xe3
#define LL_FACTORY_RESET                    0xe4
#define LL_QUICK_TERMINATION                0xe5
#define LL_RESTART_BOOTLOADER               0xe6
#define LL_NVM_ACCESS_MODE                  0xea
#define LL_HIGH_ACCURACY_MODE               0xeb
#define LL_SOC_TEMPERATURE                  0xec
#define LL_INTERNAL_STATUS_TEST             0xed
#define LL_RADIO_CONFIG                     0xf0

#define LL_I2C_SUCCESS                      0x01
#define LL_I2C_FAILURE                      0x00
#define LL_SOC_TEMPERATURE_ERROR            0x80
#define HIGH_ACCURACY_MODE_DISABLED         0x00

// definitions to handle the UNIT ID / ANT ID
#define BYTE_ONE                            0x01
#define BYTE_TWO                            0x02
#define BYTE_THREE_MSB                      0x03

#define SPI_MAX_REGISTER                    0xFF

/** @brief      Decode the payload that was received from a TWI master write
 *
 *  @details    This function should be called after a write request has completed, the TWI master (External MCU) has finished sending data.
 *              If a lidar lite register supports both read and writes, then @ref is_register_read_request specifies if the external MCU
 *              is attempting a read or write on that register
 *
 *              The message will be decoded and will take action accordingly:
 *                - If a Write register was specified, then the nRF will perform an action based on the message payload (eg. take a distance measurement)
 *                - If a Read register was specified, then the nRF will need to prepare the TWI transmit buffer immediately because a read request is imminent
 *
 * @param[in]   p_rx_data                   The payload of the TWI write
 * @param[in]   rx_data_size                The size of the payload
 * @param[in]   is_register_read_request    Specifies if the register address in the payload is a write or read request
 */
void serial_twis_parser_decode_message(uint8_t * p_rx_data, uint32_t rx_data_size, bool is_register_read_request);

void set_emitter_register(uint8_t);

#endif

