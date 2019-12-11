/*
Copyright (c) 2018 Garmin Ltd. or its subsidiaries.

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

#include <string.h>
#include "ant_message_parser.h"
#include "ant_ranging_profile.h"
#include "serial_twis.h"
#include "lidar_config_nvm.h"
#include "lidar_lite_interface.h"
#include "lidar_lite_defines.h"
#include "nrf_log.h"
#include "nrf_bootloader_info.h"

static message_page_50_template_t        m_ant_profile_page_50;

// Store the op_code
static uint8_t m_op_code = 0;

static void ant_message_process_operation(uint8_t mode);
static void ant_message_disable_i2c(ant_i2c_modes_template_t mode);
static void ant_message_enable_i2c(ant_i2c_modes_template_t mode);
static bool ant_is_power_mode_valid(uint8_t power_mode);

void ant_message_receive_configuration_setting(uint8_t *message, uint8_t simulator_page)
{
    uint8_t value;
    switch (simulator_page)
    {
        // this is generic and can handle any OP CODE in a uint8_t
        case ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS:
            // copy message into structure
            memcpy(&m_ant_profile_page_50, message, ANT_STANDARD_DATA_PAYLOAD_SIZE);
            m_op_code = m_ant_profile_page_50.op_code;
            value = m_ant_profile_page_50.value;
            ant_message_process_operation(value);
            break;

        case ANT_RP_DATA_PAGE_POWER_MODE:
        {
            #define NO_CHANGE           0x00
            #define POWER_CHANGE        0x01
            #define INTERVAL_CHANGE     0x10

            ret_code_t err_code;
            uint8_t what_changed = NO_CHANGE;

            uint8_t power_mode = lidar_nvm_get_power_mode();
            uint8_t measurement_interval = lidar_nvm_get_measurement_interval();
            if (power_mode != message[4])
            {
                what_changed |= POWER_CHANGE;
            }
            if (measurement_interval != message[7])
            {
                what_changed |= INTERVAL_CHANGE;
            }

            switch(what_changed)
            {
                case POWER_CHANGE:
                    NRF_LOG_INFO("Power level change detected");
                    if (ant_is_power_mode_valid(message[4]))
                    {
                        err_code = lidar_nvm_set_power_mode(message[4]);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_INFO("New power level could not be written to NVM");
                        }
                        else
                        {
                            // stop any ANT Period triggered distance measurements
                            ant_rp_power_mode_reset();

                            // inform library
                            err_code = lidar_lite_configure_power_mode(message[4]);
                            if (err_code == LIDAR_LITE_SUCCESS)
                            {
                                // inform ANT module
                                ant_rp_power_mode_update(message[4]);
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
                    break;
                case INTERVAL_CHANGE:
                {
                    NRF_LOG_INFO("Interval change dectected");
                    err_code = lidar_nvm_set_measurement_interval(message[7]);
                    if (err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_INFO("Measurement Interval could not be written to NVM");
                    }
                    else
                    {
                        ant_rp_measurement_interval_udpate(message[7]);
                    }
                }
                break;
                case POWER_CHANGE + INTERVAL_CHANGE:
                    NRF_LOG_INFO("Both power and interval changed");
                    if (ant_is_power_mode_valid(message[4]))
                    {
                        err_code = lidar_nvm_set_power_and_interval(message[4], message[7]);
                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_INFO("Power and Measurement Interval could not be written to NVM");
                        }
                        else
                        {
                            // stop any ANT Period triggered distance measurements
                            ant_rp_power_mode_reset();

                            // inform library
                            err_code = lidar_lite_configure_power_mode(message[4]);
                            if (err_code == LIDAR_LITE_SUCCESS)
                            {
                                // inform ANT module
                                ant_rp_power_mode_update(message[4]);
                                ant_rp_measurement_interval_udpate(message[7]);
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
                    break;
                default:
                    NRF_LOG_INFO("Power page not processed.");
                    break;
            }
        }
        break;
        default:
            break;
    }
}

static void ant_message_process_operation(uint8_t new_setting)
{
    switch(m_op_code)
    {
        case I2C_OP_CODE:
        {
            ant_i2c_modes_template_t i2c_mode = new_setting;
            uint8_t current_setting = lidar_nvm_config_get_twis_mode();
            switch(new_setting)
            {
                case I2C_ENABLED:
                    if (current_setting != I2C_ENABLED)     // only enable if currently DISABLED
                    {
                        ant_message_enable_i2c(i2c_mode);
                    }
                    break;
                case I2C_DISABLED:
                    if (current_setting != I2C_DISABLED)    // only disable if currently ENABLED
                    {
                        ant_message_disable_i2c(i2c_mode);
                    }
                    break;
                default:
                    NRF_LOG_INFO("I2C mode setting is not valid: 0x%02x", new_setting);
                    break;
            }
        }
        break;

        case FACTORY_RESET_OP_CODE:
        {
            // to be valid code must be non zero and not 0xFF
            if (new_setting != ANT_RP_INVALID_OP_CODE && new_setting != OP_CODE_SETTING_OFF)
            {
                // Pass control to NVM
                ret_code_t err_code = lidar_nvm_factory_reset();
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("Factory reset failed");
                }
            }
        }
        break;

        case QUICK_TERMINATION:
            if (new_setting == LIDAR_QUICK_TERMINATION_DISABLED || new_setting == LIDAR_QUICK_TERMINATION_ENABLED)
            {
                ret_code_t err_code = lidar_nvm_set_quick_termination_mode(new_setting);
                if (err_code == NRF_SUCCESS)
                {
                    // pass to library
                    err_code = lidar_lite_set_quick_termination_mode(new_setting);
                    if (err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Quick termination mode could not be installed in Library.");
                    }
                }
                else
                {
                    NRF_LOG_ERROR("Error storing quick terminaton in NVM");
                }

            }
            break;

        case DETECTION_SENSITIVITY: // configuration command received
        {
            ret_code_t err_code;
            uint8_t current_value;
            // We only want to store and apply new values
            current_value = lidar_nvm_get_detection_sensitivity();
            if (current_value != new_setting)
            {
                err_code = lidar_nvm_set_detection_sensitivity(new_setting);
                if (err_code == NRF_SUCCESS)
                {
                    // pass to library
                    err_code = lidar_lite_set_detection_sensitivity(new_setting);
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

        case ACQUISTION_COUNT:      // 0x4D FPGA register
        {
            NRF_LOG_INFO("Acquisition Count Op Code triggered");
            ret_code_t err_code = lidar_nvm_set_acquisition_count(new_setting);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Setting of acquisition count failed.");
            }
            lidar_lite_set_acquisition_count(new_setting);

        }
        break;

        case START_BOOTLOADER:
        {
            // to be valid code must be non zero and not 0xFF
            if (new_setting != ANT_RP_INVALID_OP_CODE && new_setting != OP_CODE_SETTING_OFF)
            {
                ret_code_t err_code;

                NRF_LOG_INFO("Boot loader restart requested");
                err_code = sd_power_gpregret_clr(0, 0xffffffff);
                APP_ERROR_CHECK(err_code);

                err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
                APP_ERROR_CHECK(err_code);
                sd_nvic_SystemReset();
            }
        }
        break;

        case HIGH_ACCURACY_OP_CODE:
        {
            NRF_LOG_INFO("High Accuracy Op Code triggered");
            ret_code_t err_code = lidar_nvm_set_high_accuracy_count(new_setting);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Setting of high accuracy count failed.");
            }
            else
            {
                // pass details to library
                lidar_lite_set_high_accuracy_count(new_setting);
            }
        }
        break;

        case NVM_MODE_OP_CODE:
        {
            NRF_LOG_INFO("NVM access mode Op Code triggered");
            if (new_setting == NVM_STORAGE_ENABLED || new_setting == NVM_STORAGE_DISABLED)
            {
                ret_code_t err_code = lidar_nvm_set_access_mode(new_setting);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("Setting of NVM access mode failed.");
                }
            }
            else
            {
                NRF_LOG_ERROR("NVM Access value is not recognized");
            }
        }
        break;

        default:
            NRF_LOG_DEBUG("Unknown Op Code detected");
            break;
    }
}

static void ant_message_disable_i2c(ant_i2c_modes_template_t mode)
{
    ret_code_t err_code;

    // this function and its sub functions do not return error codes
    serial_twis_disable();

    // need to write the new configuration byte to the NVM
    err_code = lidar_nvm_config_set_twis_mode(mode);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("NVM udate failed on TWIS disable operation");
    }
    else
    {
        sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    }
}

static void ant_message_enable_i2c(ant_i2c_modes_template_t mode)
{
    ret_code_t  err_code;

    err_code = serial_twis_configure_i2c_addresses(LL_BOTH_ADDRESSES);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_DEBUG("I2C configuration command FAILED");
    }
    else
    {
        // need to write the new configuration byte to the NVM
        err_code = lidar_nvm_config_set_twis_mode(mode);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("NVM udate failed on TWIS disable operation");
        }
        else
        {
            sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
        }
    }
}


static bool ant_is_power_mode_valid(uint8_t power_mode)
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

