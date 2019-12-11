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

#ifndef __ANT_MESSAGE_PARSER__
#define __ANT_MESSAGE_PARSER__

#include <stdint.h>
#include <stdbool.h>

#define OP_CODE_SETTING_OFF    0x00
#define OP_CODE_SETTING_ON     0x01

// Settings page layout defined in SimulANT+
// Modes for I2C
typedef enum
{
    I2C_ENABLED     = 0xF0,
    I2C_DISABLED    = 0x0F
}ant_i2c_modes_template_t;

typedef enum
{
    I2C_OP_CODE             = 0x10,
    FACTORY_RESET_OP_CODE   = 0x11,
    QUICK_TERMINATION       = 0x12,
    DETECTION_SENSITIVITY   = 0x13,
    ACQUISTION_COUNT        = 0x14,
    START_BOOTLOADER        = 0x15,
    HIGH_ACCURACY_OP_CODE   = 0x16,
    NVM_MODE_OP_CODE        = 0x17
}ant_op_code_template_t;

typedef struct
{
    uint8_t datapagenumber;
    uint8_t sequence_number;
    uint8_t spare1;
    uint8_t spare2;
    uint8_t spare3;
    uint8_t spare4;
    ant_op_code_template_t op_code;     // OP code number
    uint8_t value;                      // OP code value
}message_page_50_template_t;

/** @brief     Process the virtual register configuration message
 *
 * @param[in]   message [8 byte ANT message]
 *
 * @retval       void
*/
void ant_message_receive_configuration_setting(uint8_t *message, uint8_t page);
#endif
