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

#ifndef __ANT_RANGING_PROFILE__
#define __ANT_RANGING_PROFILE__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_sdh_ant.h"
#include "lidar_Lite_defines.h"

// LIDAR Channel Parameters
#define ANT_RP_NETWORK_NUM                    0       // Network number.
#define ANT_RP_BROADCAST_CHANNEL_NUMBER       0       // Broadcast channel number.
#define ANT_RP_CHAN_ID_DEV_TYPE               16      // Device Type.
#define ANT_RP_CHAN_ID_TRANS_TYPE_LSN         1       // Least significant nibble of the Transmission type
#define ANT_RP_CHAN_PERIOD                    8192    // Channel Period (in 32 kHz counts).
#define ANT_RP_RF_FREQ                        66      // RF Frequency.

// Generic Defines
#define ANT_RP_RESERVED_ZEROS                 0x00
#define ANT_RP_RESERVED_ONES                  0xFF
#define ANT_RP_NO_MEASAUREMENTS               0x00    // The Lidar device has not taken a distance measurement yet
#define ANT_RP_MEASUREMENT_FAILED             0x01    // The Lidar device was unable to return a distance measurement
#define ANT_RP_INVALID_OP_CODE                0xFF
#define GARMIN_MANUFACTURER_ID                0x01    // From the FIT SDK
#define GARMIN_PRODUCT_NUMBER                 3443    // From the FIT SDK  0xD73
#define ANT_SOC_TEMPERATURE_ERROR             0x80    // Default value for unsuccessful temperature

// LIDAR Data Pages
#define ANT_RP_DATA_PAGE_MEASUREMENT_DATA     0x10      // default data page
#define ANT_LOW_PRIORITY_MESSAGE              0x05
#define ANT_HIGH_PRIORITY_MESSAGE             0x10
#define PAGE_ONE_INTERLEAVE                   65        // send message after every 64 default data pages
#define PAGE_TWO_INTERLEAVE                   130       // send message after every 64 + 1 + 64 pages.

#define ANT_RP_DATA_PAGE_POWER_MODE             0x30
#define ANT_RP_DATA_PAGE_TRIGGER_MEASUREMENT    0x31
#define ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS     0xF0
#define ANT_RP_DATA_PAGE_TRANS_REQUEST          0x46
#define ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE   0x50
#define ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE        0x51

// priority queue structure
typedef struct
{
    uint8_t priority;
    uint8_t data_page;
    uint8_t status;     // see below for bit definitions
    uint8_t tx_count;   // number of transmissions
    uint8_t payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];
}priority_queue_t;

#define PRIORITY_QUEUE_SIZE     5
// #defines for status byte in priority_queue_t
#define ANT_REGULAR_BROADCAST_MESSAGE   0x00
#define ANT_ACKNOWLEDGE_REQUESTED       0x01    // bit zero
#define ANT_ACKNOWLEDGE_TILL_PASS       0x10    // bit four
#define ANT_MESSAGE_ACKNOWLEDGE_MASK    0x80
#define ANT_MAX_7_BIT_COUNT_MASK        0x7F


 /**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
void ant_rp_evt_handler(ant_evt_t * p_ant_evt, void * p_context);

/**@brief   Function for setting up the ANT module to be ready for ANT communication.
 *
 * @details This function configures the ANT ranging profile channel and
 *          opens the channel to begin transmissions.
 */
void ant_rp_init(void);

/**@brief   Update the distance transmitted over ANT.
 *
 * @details This function updates the distance transmitted on data page 16
 *          The event count will be incremented each time this function is called.
 *
 * @param[in] distance_cm   The distance in centimeters
 */
void ant_rp_update_distance(uint16_t distance_cm);

/** @brief      Sets the last distance measurement to error
 *
 * @details     When ANT detects the op code to take a measurement the value
 *              parameter must either be a 0x01 or 0x02.  Anything else is not
 *              valid.  This function will set the local variable in
 *              ant_ranging_profile.c to ANT_RP_MEASUREMENT_FAILED (0x01)
 *              This is visual feedback to the Simulator user
 *
 * @retval       void
*/
void ant_rp_set_distance_error();

/**@brief   Update the power mode level received from I2C or ANT
 *
 * @details This function updates the local power_mode in the ant_ranging_profile module
 *
 * @param[in] power_mode   Enumerated type: SYNCH, ASYNCH, ALWAYS_ON
 */
void ant_rp_power_mode_update(lidar_lite_power_config_t power_mode);

/**@brief   Reset the power level to ALWAYS_ON
 *
 * @details This function resets the local power_mode in the ant_ranging_profile module
 */
void ant_rp_power_mode_reset();

/**@brief   Update the measurement interval delay received from I2C or ANT
 *
 * @details This function updates the local interval delay in the ant_ranging_profile module
 *
 * @param[in] interval  0 - 255
 */
void ant_rp_measurement_interval_udpate(uint8_t interval);

#endif
