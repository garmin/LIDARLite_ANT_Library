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

#include "serial_twis_parser.h"
#include "serial_twis.h"
#include "sdk_config.h"
#include "lidar_lite_interface.h"
#include "lidar_config_nvm.h"
#include "ant_ranging_profile.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_bootloader_info.h"
#include "nrf_dfu_types.h"
#include "app_interface.h"

#define NRF_LOG_MODULE_NAME ll_serial_twis_parser
#if NRF_LOG_ENABLED && LL_TWI_PARSER_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       LL_TWI_PARSER_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL       0
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define LL_TWI_PARSER_TX_MESSAGE_SIZE       MAX(LL_MAX_REGISTER_SIZE_BYTES, LIDAR_LITE_LIB_VERSION_LENGTH)      // Should be equal to size of the largest message to be sent over I2C
#define I2C_SEC_ADDRESS_BUFFER_SIZE         5
#define MAX_STATUS_SIZE                     20  // used in internal status testing

// local method to check enumerated type
static bool serial_twis_is_power_mode_valid(uint8_t mode);

// two definitions that map from the ANT module
#define LL_INVALID_BYTE                     ANT_RP_RESERVED_ZEROS
#define LL_DISABLED_VALUE                   ANT_RP_RESERVED_ONES

void serial_twis_parser_decode_message(uint8_t * p_rx_data, uint32_t rx_data_size, bool is_register_read_request)
{

    if (p_rx_data == NULL || rx_data_size == 0)
    {
        NRF_LOG_ERROR("Invalid received message");
        return;
    }

    uint8_t     register_address   = p_rx_data[0];
    uint8_t     set_value;
    uint8_t     command            = 0;
    uint8_t     tx_data[LL_TWI_PARSER_TX_MESSAGE_SIZE];
    ret_code_t  err_code;

    switch (register_address)
    {
        case LL_REGISTER_ACQ_COMMAND:
        {
            command = p_rx_data[1];

            if (is_register_read_request == false)
            {
                switch (command)
                {
                    case ACQ_COMMAND_TAKE_DISTANCE:
                        NRF_LOG_DEBUG("Take Distance Command");
                        if (lidar_lite_request_measurement(false) == LIDAR_LITE_SUCCESS)  // Take distance measurement using only range pulses only
                        {
                            measurement_started();
                        }
                        break;
                    case ACQ_COMMAND_TAKE_DISTANCE_BIAS:
                        NRF_LOG_DEBUG("Take Distance (w/ bias correction) Command");
                        if (lidar_lite_request_measurement(true) == LIDAR_LITE_SUCCESS)   // Take distance measurement using only range pulses and the reference LED
                        {
                            measurement_started();
                        }
                        break;
                }
            }
            break;
        }

        case LL_REGISTER_STATUS:
        {
            // Attempt to read the status register directly from the FPGA
            lidar_lite_return_t ll_ret_code = lidar_lite_request_register_read(LL_REGISTER_STATUS);

            // If in low power mode, let the I2C interface know so that they can slow transmissions
            uint8_t power_mode = lidar_nvm_get_power_mode();
            tx_data[0] = 0;

            if (power_mode != LIDAR_LITE_ALWAYS_ON)
            {
                tx_data[0] |= LL_STATUS_INTERFACE_LOW_PWR;
            }

            // If the library is currently busy, then report that over the TWI interface
            if (ll_ret_code != LIDAR_LITE_SUCCESS)
            {
                //Report that the library is currently taking a distance measurement or performing an I2C request in low power mode
                tx_data[0] |= LL_STATUS_BUSY;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                // cached version, this becomes stale very quickly as the Automatic Noise
                // adjustment is running all the time on the FPGA and BUSY is common when reading
                // This register
                tx_data[0] |= lidar_lite_get_library_register_value(LL_REGISTER_STATUS);
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            break;
        }

        case LL_REGISTER_ACQUISITION_COUNT:
        {
            if (is_register_read_request)
            {
                uint8_t acquisition_count = lidar_nvm_get_acquisition_count();
                tx_data[0] = acquisition_count;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                ret_code_t err_code;
                uint8_t new_acquisition_count;
                new_acquisition_count = p_rx_data[1]; // pick up the interval

                uint8_t current_count = lidar_nvm_get_acquisition_count();
                // only change interval if different from stored setting
                if (current_count != new_acquisition_count)
                {
                    err_code = lidar_nvm_set_acquisition_count(new_acquisition_count);
                    if (err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Acquistion count could not be written to NVM");
                    }
                    else
                    {
                        NRF_LOG_INFO("Acqusition Count Written to NVM => %d",new_acquisition_count);
                        lidar_lite_set_acquisition_count(new_acquisition_count);
                    }
                }
            }
            break;
        }

        case LL_REGISTER_FULL_DELAY_LOW:
        {
            lidar_lite_retrieve_last_measurement(&tx_data[1], &tx_data[0]);
            NRF_LOG_DEBUG("<%d> Last measured distance: %02x, %02x",__LINE__, tx_data[0], tx_data[1]);

            // Prepare two bytes of data so that TWI master can use auto increment to get full distance
            serial_twis_prepare_tx_buffer(tx_data, 2);
            break;
        }

        case LL_REGISTER_FULL_DELAY_HIGH:
        {

            uint8_t unused_low_byte;
            lidar_lite_retrieve_last_measurement(&tx_data[0], &unused_low_byte);

            // Prepare two bytes of data so that TWI master can use auto increment to get full distance
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        // If a register read from address 0x16 is intercepted, return 4 byte UNIT ID
        // If a register write from address 0x16 is intercepted, accept 5 byte command
        case LL_REGISTER_UNIT_ID_0:
        {
            if (is_register_read_request)
            {
                uint8_t unit_id[4] = {0};
                // need to retrieve the 4 byte ANT ID, this is little endian
                serial_twis_get_unit_identification(unit_id);
                memcpy(&tx_data[0], unit_id, 4);
                serial_twis_prepare_tx_buffer(tx_data, 4);
            }
            else        // write request
            {
                uint8_t buffer[I2C_SEC_ADDRESS_BUFFER_SIZE] = {0};
                memcpy(buffer, &p_rx_data[1], I2C_SEC_ADDRESS_BUFFER_SIZE);
                err_code = serial_twis_check_and_configure_secondary_i2c_address(buffer);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_DEBUG("Secondary I2C address not applied");
                }

            }
            break;
        }

        case LL_REGISTER_UNIT_ID_1:
        {
            uint8_t unit_id[4] = {0};
            // need to retrieve byte 1 from Unit ID
            serial_twis_get_unit_identification(unit_id);

            tx_data[0] = unit_id[BYTE_ONE];
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_REGISTER_UNIT_ID_2:
        {
            uint8_t unit_id[4] = {0};
            // need to retrieve byte 2 from Unit ID
            serial_twis_get_unit_identification(unit_id);

            tx_data[0] = unit_id[BYTE_TWO];
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_REGISTER_UNIT_ID_3:
        {
            uint8_t unit_id[4] = {0};
            // need to retrieve high byte, byte 3 from Unit ID
            serial_twis_get_unit_identification(unit_id);

            tx_data[0] = unit_id[BYTE_THREE_MSB];
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_REGISTER_I2C_SEC_ADDR:
        {
            err_code = lidar_nvm_config_get_slave_address(&tx_data[0]);
            if (err_code == NRF_SUCCESS)
            {
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                NRF_LOG_DEBUG("Secondary I2C slave address not found in NVM");
            }
            break;
        }

        case LL_REGISTER_I2C_CONFIG:
        {
            command = p_rx_data[1];
            err_code = serial_twis_configure_i2c_addresses(command);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("I2C configuration command FAILED");
            }
            break;
        }

        case LL_REGISTER_DETECTION_SENSITIVITY:
        {
            if (is_register_read_request)
            {
                uint8_t detection_sensitivity;
                // read value from NVM
                detection_sensitivity = lidar_nvm_get_detection_sensitivity();
                tx_data[0] = detection_sensitivity;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                // We only want to store and apply new values
                uint8_t current_value = lidar_nvm_get_detection_sensitivity();
                command = p_rx_data[1];     // pick up value for the sensitivity level
                if (current_value != command)
                {
                    err_code = lidar_nvm_set_detection_sensitivity(command);
                    if (err_code == NRF_SUCCESS)
                    {
                        // pass to library
                        err_code = lidar_lite_set_detection_sensitivity(command);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_ERROR("Dectection sensitivity could not be installed in Library");
                        }
                    }
                    else
                    {
                        NRF_LOG_ERROR("Error storing detection sensitivity in NVM");
                    }
                }
            }
            break;
        }

        case LL_REGISTER_LIB_VERSION:
        {
            NRF_LOG_DEBUG("Request Lib Version");
            const char * lib_version;

            err_code = lidar_lite_request_library_version(&lib_version);
            APP_ERROR_CHECK(err_code);

            memcpy(tx_data, lib_version, LIDAR_LITE_LIB_VERSION_LENGTH);
            serial_twis_prepare_tx_buffer(tx_data, LIDAR_LITE_LIB_VERSION_LENGTH);
            break;
        }

        case LL_REGISTER_CORR_DATA:
        {
            if (is_register_read_request)
            {
                int16_t corr_data;
                lidar_lite_get_element_from_correlation_record(&corr_data);
                tx_data[0] = (uint8_t)(corr_data & 0xFF);
                tx_data[1] = (uint8_t)(corr_data >> 8);
                serial_twis_prepare_tx_buffer(tx_data, 2);
            }
            else
            {
                // call to reset internal pointer to the beginning of the correlation memory address
                // function call always returns LIDAR_LITE_SUCCESS
                lidar_lite_reset_correlation_record_pointer();
            }
            break;
        }

        case LL_REGISTER_GVER_LO:
        {
            // trigger a new FPGA read of low version number
            lidar_lite_request_fpga_version_low();
            NRF_LOG_DEBUG("Request Garmin Version Low");

            // retrieve last read from the cache
            tx_data[0] = lidar_lite_get_library_register_value(LL_REGISTER_GVER_LO);
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
       }

        case LL_REGISTER_GVER_HI:
        {
            // trigger a new FPGA read of high version number
            lidar_lite_request_fpga_version_high();
            NRF_LOG_DEBUG("Request Garmin Version High");

            // retrieve last read from the cache
            tx_data[0] = lidar_lite_get_library_register_value(LL_REGISTER_GVER_HI);
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_TEMP_MEASUREMENT:
        {
            // Kick off temperature measurement, TWI buffer will be loaded when main receives the temperature event
            lidar_lite_get_temp();
            // retrieve the last board temperature reading
            tx_data[0] = lidar_lite_retrieve_board_temperature();
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_HARDWARE_VERSION:
        {
            // hardware version is known right after FPGA powers up
            tx_data[0] = lidar_lite_retrieve_hardware_version();
            serial_twis_prepare_tx_buffer(tx_data, 1);
            break;
        }

        case LL_POWER_MODE:
        {
            NRF_LOG_INFO("Power Mode register called.");
            if (is_register_read_request)
            {
                uint8_t power_mode = lidar_nvm_get_power_mode();
                tx_data[0] = power_mode;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                ret_code_t err_code;
                uint8_t new_power_mode;
                // read power level value
                new_power_mode = p_rx_data[1];

                if (lidar_nvm_get_high_accuracy_count() == HIGH_ACCURACY_MODE_DISABLED)
                {
                    // get current power mode
                    uint8_t power_mode = lidar_nvm_get_power_mode();
                    // only change power level if different from stored setting
                    if (power_mode != new_power_mode)
                    {
                        if (serial_twis_is_power_mode_valid(new_power_mode))
                        {
                            err_code = lidar_nvm_set_power_mode(new_power_mode);
                            if (err_code != NRF_SUCCESS)
                            {
                                NRF_LOG_INFO("New power Mode could not be written to NVM");
                            }
                            else
                            {
                                // stop any ANT Period triggered distance measurements
                                ant_rp_power_mode_reset();

                                // inform library
                                err_code = lidar_lite_configure_power_mode(new_power_mode);
                                if (err_code == LIDAR_LITE_SUCCESS)
                                {
                                    // inform ANT module
                                    ant_rp_power_mode_update(new_power_mode);
                                }
                                else
                                {
                                    NRF_LOG_INFO("Power mode could not be applied to system: %d",err_code);
                                }
                            }
                        }
                        else
                        {
                            NRF_LOG_ERROR("Power mode setting is not valid.");
                        }
                    }
                }
                else
                {
                    NRF_LOG_INFO("Power mode could not be applied because High Accuracy Enabled");
                }
            }
            break;
        }

        case LL_MEASUREMENT_INTERVAL:
        {
            NRF_LOG_INFO("Measurement Interval register called.");
            if (is_register_read_request)
            {
                uint8_t measurement_interval = lidar_nvm_get_measurement_interval();
                tx_data[0] = measurement_interval;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                ret_code_t err_code;
                uint8_t new_interval;
                new_interval = p_rx_data[1]; // pick up the interval

                if (lidar_nvm_get_high_accuracy_count() == HIGH_ACCURACY_MODE_DISABLED)
                {
                    uint8_t measurement_interval = lidar_nvm_get_measurement_interval();
                    // only change interval if different from stored setting
                    if (measurement_interval != new_interval)
                    {
                        err_code = lidar_nvm_set_measurement_interval(new_interval);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_INFO("Measurement Interval could not be written to NVM");
                        }
                        else
                        {
                            NRF_LOG_INFO("Interval => %d",new_interval);
                            // need to send this information to the ANT module
                            // this will be processed and validated there
                            ant_rp_measurement_interval_udpate(new_interval);
                        }
                    }
                }
                else
                {
                    NRF_LOG_INFO("Measurement Interval could not be applied because High Accuracy Enabled");
                }
            }
            break;
        }

        case LL_FACTORY_RESET:
        {
            if (is_register_read_request == false)
            {
                // check what value was received first
                command = p_rx_data[1];
                // to be valid code must be non zero and not 0xFF
                if (command != LL_INVALID_BYTE && command != LL_DISABLED_VALUE)
                {
                    ret_code_t err_code = lidar_nvm_factory_reset();
                    if (err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Factory reset failed");
                    }
                }
            }
            break;
        }

        case LL_QUICK_TERMINATION:
        {
            if (is_register_read_request)
            {
                uint8_t termination_mode = lidar_nvm_get_quick_termination_mode();
                tx_data[0] = termination_mode;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                ret_code_t err_code;
                uint8_t new_termination_mode;

                new_termination_mode = p_rx_data[1]; // get new value from interface
                if (new_termination_mode != LIDAR_QUICK_TERMINATION_DISABLED && new_termination_mode != LIDAR_QUICK_TERMINATION_ENABLED)
                {
                    NRF_LOG_ERROR("Invalid termination mode detected");
                }
                else
                {
                    uint8_t current_mode = lidar_nvm_get_quick_termination_mode();
                    if (current_mode != new_termination_mode)
                    {
                        err_code = lidar_nvm_set_quick_termination_mode(new_termination_mode);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_ERROR("Termination mode could not be written to NVM");
                        }
                        else
                        {
                            err_code = lidar_lite_set_quick_termination_mode(new_termination_mode);
                            if (err_code != NRF_SUCCESS)
                            {
                                NRF_LOG_ERROR("Termination mode could not be set in Library");
                            }
                        }
                    }
                }
            }
            break;
        }

        case LL_RESTART_BOOTLOADER:
        {
            if (is_register_read_request == false)
            {
                // check what value was received first
                command = p_rx_data[1];
                // to be valid code must be non zero and not 0xFF
                if (command != LL_INVALID_BYTE && command != LL_DISABLED_VALUE)
                {
                    NRF_LOG_INFO("Boot loader restart requested");
                    err_code = sd_power_gpregret_clr(0, 0xffffffff);
                    APP_ERROR_CHECK(err_code);

                    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
                    APP_ERROR_CHECK(err_code);
                    sd_nvic_SystemReset();
                }
            }
            break;
        }

        case LL_NVM_ACCESS_MODE:
        {
            if (is_register_read_request == true)
            {
                bool mode = lidar_nvm_get_access_mode();
                tx_data[0] = (mode == false) ? NVM_STORAGE_DISABLED : NVM_STORAGE_ENABLED;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else
            {
                bool command_mode;
                bool error_flag = false;
                bool current_mode = lidar_nvm_get_access_mode();

                if (p_rx_data[1] == NVM_STORAGE_DISABLED)
                {
                    command_mode = false;
                }
                else if (p_rx_data[1] == NVM_STORAGE_ENABLED)
                {
                    command_mode = true;
                }
                else
                {
                    NRF_LOG_ERROR("Invalid setting for NVM access");
                    error_flag = true;
                }

                if (current_mode != command_mode && error_flag == false)
                {
                    err_code = lidar_nvm_set_access_mode(p_rx_data[1]);
                    if (err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Access Mode could not be written to NVM");
                    }
                    else
                    {
                        NRF_LOG_INFO("Access Mode written to NVM => %d",p_rx_data[1]);
                    }
                }
            }
            break;
        }

        case LL_HIGH_ACCURACY_MODE:
        {
            if (is_register_read_request)
            {
                uint8_t counter = lidar_nvm_get_high_accuracy_count();
                tx_data[0] = counter;
                serial_twis_prepare_tx_buffer(tx_data, 1);
            }
            else    // write request
            {
                ret_code_t ret;

                // only install the High accuracy mode if the power is in the correct mode
                uint8_t power_mode = lidar_nvm_get_power_mode();
                if (power_mode == LIDAR_LITE_ALWAYS_ON)
                {
                    uint8_t new_setting = p_rx_data[1];
                    uint8_t current_setting = lidar_nvm_get_high_accuracy_count();

                    // we only want to write the NVM if a newer value has been set
                    if (new_setting != current_setting)
                    {
                        err_code = lidar_nvm_set_high_accuracy_count(new_setting);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_ERROR("High accuracy count could not be written to NVM");
                        }
                        else
                        {
                            NRF_LOG_INFO("High accuracy count written to NVM => %d",new_setting);
                            lidar_lite_set_high_accuracy_count(new_setting);
                        }
                    }
                }
                else
                {
                    NRF_LOG_INFO("High accuracy could not be enabled, wrong power mode");
                }
            }
            break;
        }

        case LL_SOC_TEMPERATURE:
        {
            if (is_register_read_request)
            {
                uint8_t enabled;
                int32_t temp;

                sd_softdevice_is_enabled(&enabled);
                if (enabled)
                {
                    if (sd_temp_get(&temp) == NRF_SUCCESS)
                    {
                        temp = (int8_t)(temp / 4);   // from examples in SDK 15, need to divide by 4
                        NRF_LOG_INFO("<%d> SoC temperature: %d",__LINE__,temp);
                        tx_data[0] = (uint8_t)temp;
                    }
                    else    // error reading temperature
                    {
                        tx_data[0] = LL_SOC_TEMPERATURE_ERROR;
                    }
                    serial_twis_prepare_tx_buffer(tx_data, 1);
                }
                else
                {
                    // softdevice is not enabled.
                    NRF_LOG_ERROR("Softdevice required to read SoC temperature");
                }
            }
            break;
        }


        default:
            if (register_address <= SPI_MAX_REGISTER)
            {
                if (is_register_read_request)
                {
                    uint8_t value;
                    // trigger a register read
                    lidar_lite_request_register_read(register_address);
                    value = lidar_lite_get_library_register_value(register_address);

                    NRF_LOG_DEBUG("Generic register read on address 0x%02x", register_address);

                    tx_data[0] = value;
                    serial_twis_prepare_tx_buffer(tx_data, 1);
                }
                else
                {
                    command = p_rx_data[1];

                    NRF_LOG_DEBUG("Generic register write on address 0x%02x, with value 0x%02x", register_address, command);
                    lidar_lite_request_register_write(register_address, command);
                }
            }
            else
            {
                NRF_LOG_WARNING("Requested register is out of bounds: 0x%02X, Max allowable: 0x%02X",register_address, SPI_MAX_REGISTER);
            }
            break;
    }

    return;
}

static bool serial_twis_is_power_mode_valid(uint8_t power_mode)
{
    bool valid;
    switch(power_mode)
    {
        case LIDAR_LITE_ASYNCHRONOUS_MODE:
        case LIDAR_LITE_SYNCHRONOUS_MODE:
        case LIDAR_LITE_ALWAYS_ON:
            valid = true;
            break;
        default:
            valid = false;
            break;
    }
    return valid;
}

