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

#include <string.h>
#include <stdlib.h>     // for qsort()
#include "ant_ranging_profile.h"
#include "ant_message_parser.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "nrf_soc.h"    // for SOC temperature
#include "lidar_lite_interface.h"
#include "app_interface.h"
#include "lidar_config_nvm.h"   // access to NVM

#define NRF_LOG_MODULE_NAME ant_rp
#if NRF_LOG_ENABLED && ANT_RP_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       ANT_RP_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL       0
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/** @brief
    The network key is an 8-byte number that uniquely identifies a network. Only channels with identical valid network keys may communicate with each other. Only valid
    network keys will be accepted by ANT, so if attempting to set the Network Key with an invalid key, the network key will not be changed; it will retain the value it
    held prior to the command. The public network key provided below is open to all ANT capable devices and has no set rules governing its use, if creating a custom
    implementation the public key can be used. Private network keys are also available at https://www.thisisant.com
    For more information about network keys, refer to the ANT Message Protocol and Usage document available at https://www.thisisant.com/developer/resources/downloads

    Steps to enable ANT network key:
        1. Uncomment ENABLE_ANT_RP_NETWORK_KEY to enable ANT network key and turn off compilation error
        2. Choose ANT network key based on your applications use case:
            a) Default ANT implementations:
                - The LLv4 comes programmed from the factory with the Garmin Developer Key. To reprogram the LLv4 with the Garmin Developer Key so that the LLv4 can be
                  used with the provided simulator, you will need to replace the Public Key provided below with the Garmin Developer Key. Delete the public key below
                  and copy/paste with the Garmin Developer Key. The use of the Garmin Developer Key comes with restrictions, therefore, to gain access to the Garmin
                  Developer Key you will need to follow the usage notice located here: https://www.thisisant.com/developer/ant-plus/ant-plus-basics/network-keys
            b) Custom ANT implementations:
                - If the ANT implementation is modified, then the Garmin Developer Key can not be used. The public key provided below can be used during development.
                  Private keys are recommended for commercial implementations.

*/
//#define ENABLE_ANT_RP_NETWORK_KEY
#ifdef  ENABLE_ANT_RP_NETWORK_KEY
    const uint8_t ANT_RP_NETWORK_KEY[] = {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1}; // Public Key
#endif
#ifndef ENABLE_ANT_RP_NETWORK_KEY
    #error "Must enable ANT network key. To enable the ANT network key, follow the steps in description above."
#endif

static void ant_rp_channel_setup(void);
static int8_t ant_rp_soc_temperature();

static void ant_rp_message_send(uint8_t * message_payload, uint8_t status);
static void ant_rp_prepare_data_page_16(uint8_t *message_payload);
static void ant_rp_prepare_data_page_80(uint8_t *message_payload);
static void ant_rp_prepare_data_page_81(uint8_t *message_payload);
static void ant_rp_prepare_next_data_page(uint8_t * message_payload, uint8_t page);
static void ant_rp_process_rx_message(uint8_t *message);

// Local variable for the power level operation
static lidar_lite_power_config_t m_ANT_power_mode = LIDAR_LITE_ALWAYS_ON;   // default settiing, disabled
static uint8_t m_distance_measurement_counter = 0;
static uint8_t m_distance_measurement_interval = ANT_RP_RESERVED_ONES;

// ANT Payload
static uint8_t    m_message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];

// Priority queue functions
static priority_queue_t m_ant_priority_queue[PRIORITY_QUEUE_SIZE] = {0};
static void ant_load_record_in_queue(uint8_t priority, uint8_t *payload, uint8_t page_number, uint8_t status, uint8_t copies);
static int8_t ant_get_empty_record_from_queue();
static int8_t ant_get_priority_queue_size();
static void ant_pop_record_from_queue(priority_queue_t *record);
static void ant_sort_priority_queue();
static int ant_sort_compare_function(const void *a, const void *b);
static void ant_rp_priority_queue_processing();
static void ant_flush_priority_queue();
static void ant_rp_insert_failed_message();
static void ant_rp_store_dp_request_in_priority_queue(uint8_t *message, uint8_t page);

// Create one record that will store the special case when a retry until Acknowledgment is requested
static priority_queue_t m_copy_of_record;

// Data Page 16 Variables
static uint8_t    m_event_counter             = 0;
static uint16_t   m_last_measured_distance_cm = ANT_RP_NO_MEASAUREMENTS;

// Sequence number, populate from Page configuration requests from SimulANT
static uint8_t m_sequence_number = 0;   // start high as the simulator will start from zero
static uint8_t m_page_interleave = 0;   // for interleaving two common data pages

void ant_rp_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    if (p_ant_evt->channel == ANT_RP_BROADCAST_CHANNEL_NUMBER)
    {
        switch (p_ant_evt->event)
        {
            case EVENT_TX:
            case EVENT_TRANSFER_TX_COMPLETED:
                // update the interleave counter
                m_page_interleave++;
                if (ant_get_priority_queue_size() == 0)
                {
                    if (m_page_interleave == PAGE_ONE_INTERLEAVE)
                    {
                        ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE);
                    }
                    else if (m_page_interleave == PAGE_TWO_INTERLEAVE)
                    {
                        ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE);
                        m_page_interleave = 0;   // reset because we are at MAX
                    }
                    else
                    {
                        ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_MEASUREMENT_DATA);
                    }
                    // send message, no acknowledge required
                    ant_rp_message_send(m_message_payload, ANT_REGULAR_BROADCAST_MESSAGE);
                }
                else
                {
                    ant_rp_priority_queue_processing();
                    // if we are busy we could miss the reset of the interleave counter, check status
                    if (m_page_interleave >= PAGE_TWO_INTERLEAVE)
                    {
                        m_page_interleave = 0;
                    }
                }

                // support the synchronous mode
                if (m_ANT_power_mode == LIDAR_LITE_SYNCHRONOUS_MODE)
                {
                    // Code to trigger a measurement every interval
                    if (m_distance_measurement_interval != ANT_RP_RESERVED_ONES)
                    {
                        m_distance_measurement_counter++;
                        if (m_distance_measurement_counter % m_distance_measurement_interval == 0)
                        {
                            if (lidar_lite_request_measurement(true) == LIDAR_LITE_SUCCESS)
                            {
                                measurement_started();
                            }
                            m_distance_measurement_counter = 0;     // reset
                        }
                    }
                    else    // code to trigger a measurement if interval is not set
                    {
                        if (lidar_lite_request_measurement(true) == LIDAR_LITE_SUCCESS)
                        {
                            measurement_started();
                        }
                    }
                }
                break;

            case EVENT_RX:
                // need to insert the received message into the priority queue
                ant_flush_priority_queue();
                ant_rp_process_rx_message(p_ant_evt->message.ANT_MESSAGE_aucPayload);
                break;

            case EVENT_TRANSFER_TX_FAILED:
                // if event failed then check the priority of the "copy", if it is set then
                // need to insert back into priority queue to try again
                if (m_copy_of_record.priority == ANT_HIGH_PRIORITY_MESSAGE)
                {
                    NRF_LOG_INFO("Acknowledge failed, Priority queue insert");
                    // need to resend message
                    ant_rp_insert_failed_message();
                    // process queue for next EVENT_TX
                    ant_rp_priority_queue_processing();
                }
                else
                {
                    NRF_LOG_INFO("Acknowledge failed, Transmit until successful not requested");
                }
                break;

            default:
                NRF_LOG_ERROR("<%d> Unrecognized event in ant_rp_evt_handler(): %d",__LINE__,p_ant_evt->event);
                break;
        }
    }
}

void ant_rp_init(void)
{
    ret_code_t err_code;
    err_code = sd_ant_network_address_set(ANT_RP_NETWORK_NUM, ANT_RP_NETWORK_KEY);
    APP_ERROR_CHECK(err_code);

    ant_rp_channel_setup();
}

void ant_rp_update_distance(uint16_t distance_cm)
{
    m_last_measured_distance_cm = distance_cm;

    // m_event_counter (type uint8_t) will rollover every 256 as expected
    m_event_counter++;
}

/**@brief Function for setting up the ANT module to be ready for TX broadcast.
 */
static void ant_rp_channel_setup(void)
{
    uint32_t ant_id;
    uint16_t device_number;
    uint8_t tx_type;

    // Derive channel parameters from ANT ID stored in module
    ant_id = get_ant_id();
    // Use 20-bit device ID, where the 16 lsb go to Device Number field
    device_number = ant_id & 0xFFFF;
    if(!device_number)  // Ensure device number is different from zero
    {
        device_number = 1;
    }

    // Use the next 4 lsb of the ANT ID as the upper nibble of the Transmission Type
    // Ensure that the lower nibble of the Transmission Type specified in ant_ranging_profile.h does not exceed 4 bits
    APP_ERROR_CHECK_BOOL(ANT_RP_CHAN_ID_TRANS_TYPE_LSN <= 0x0F);
    uint8_t ext_device_id = (ant_id >> 16) & 0X0F;
    tx_type = ANT_RP_CHAN_ID_TRANS_TYPE_LSN | (ext_device_id << 4);

    NRF_LOG_INFO("Configuring ANT channel, Device number: %d, Tx Type: %d, Device Type: %d", device_number, tx_type, ANT_RP_CHAN_ID_DEV_TYPE);

    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = ANT_RP_BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_MASTER,
        .ext_assign        = 0x00,
        .rf_freq           = ANT_RP_RF_FREQ,
        .transmission_type = tx_type,
        .device_type       = ANT_RP_CHAN_ID_DEV_TYPE,
        .device_number     = device_number,
        .channel_period    = ANT_RP_CHAN_PERIOD,
        .network_number    = ANT_RP_NETWORK_NUM,
    };

    ret_code_t err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Initialize the ANT payload with the appropriate data page
    ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_MEASUREMENT_DATA);

    // Fill tx buffer for the first frame.
    ant_rp_message_send(m_message_payload, ANT_REGULAR_BROADCAST_MESSAGE);

    // Open channel.
    err_code = sd_ant_channel_open(ANT_RP_BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting payload for ANT message and sending it.
 */
static void ant_rp_message_send(uint8_t * message_payload, uint8_t status)
{

    if ( status != ANT_REGULAR_BROADCAST_MESSAGE)
    {
        // This function is used to send an acknowledge message. This message requests an acknowledgement from the slave to validate reception.
        ret_code_t err_code = sd_ant_acknowledge_message_tx(ANT_RP_BROADCAST_CHANNEL_NUMBER,
                                                            ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                            message_payload);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // Broadcast the data.
        ret_code_t err_code = sd_ant_broadcast_message_tx(ANT_RP_BROADCAST_CHANNEL_NUMBER,
                                                          ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                          message_payload);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for selecting what LIDAR data page to send next.
 */
static void ant_rp_prepare_next_data_page(uint8_t *message_payload, uint8_t data_page)
{
    switch(data_page)
    {
        case ANT_RP_DATA_PAGE_MEASUREMENT_DATA:
            ant_rp_prepare_data_page_16(message_payload);
            break;

        case ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE:
            ant_rp_prepare_data_page_80(message_payload);
            break;
        case ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE:
            ant_rp_prepare_data_page_81(message_payload);
            break;
        default:
            NRF_LOG_WARNING("Attempting to prepare unknown data page %d", data_page);
            break;
    }
}

/**@brief Function for preparing the transmit buffer to LIDAR data page 16.
 */
static void ant_rp_prepare_data_page_16(uint8_t *message_payload)
{
    message_payload[0] = ANT_RP_DATA_PAGE_MEASUREMENT_DATA;
    message_payload[1] = ANT_RP_RESERVED_ZEROS;
    message_payload[2] = ANT_RP_RESERVED_ZEROS;
    message_payload[3] = ant_rp_soc_temperature();
    message_payload[4] = ANT_RP_RESERVED_ONES;
    message_payload[5] = m_event_counter;
    message_payload[6] = (uint8_t)(m_last_measured_distance_cm & 0x00FF);
    message_payload[7] = (uint8_t)((m_last_measured_distance_cm & 0xFF00) >> 8);
}

/**@brief Function for preparing the transmit buffer to LIDAR data page 80.
 */
static void ant_rp_prepare_data_page_80(uint8_t *message_payload)
{
    message_payload[0] = ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE;
    message_payload[1] = ANT_RP_RESERVED_ONES;
    message_payload[2] = ANT_RP_RESERVED_ONES;
    message_payload[3] = lidar_lite_retrieve_hardware_version();
    message_payload[4] = GARMIN_MANUFACTURER_ID;
    message_payload[5] = ANT_RP_RESERVED_ZEROS;                 // MSB of Garmin Manufacturer is Zero
    message_payload[6] = GARMIN_PRODUCT_NUMBER & 0x00FF;        // low byte
    message_payload[7] = (GARMIN_PRODUCT_NUMBER >> 8) & 0xFF;   // high byte
}

/**@brief Function for preparing the transmit buffer to LIDAR data page 81.
 */
static void ant_rp_prepare_data_page_81(uint8_t *message_payload)
{
    uint32_t id = get_ant_id();

    message_payload[0] = ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE;
    message_payload[1] = ANT_RP_RESERVED_ONES;
    message_payload[2] = ANT_RP_RESERVED_ONES;
    message_payload[3] = lidar_lite_retrieve_fpga_version();
    message_payload[4] = id & 0x000000FF;
    message_payload[5] = (id >> 8) & 0xFF;
    message_payload[6] = (id >> 16) & 0xFF;
    message_payload[7] = (id >> 24) & 0xFF;
}

static int8_t ant_rp_soc_temperature()
{
    int32_t temp;

    if (sd_temp_get(&temp) == NRF_SUCCESS)
    {
        return (int8_t)(temp / 4);
    }
    else    // temperature could not be done
    {
        return ANT_SOC_TEMPERATURE_ERROR;
    }
}

// this is called from the ant_rp_event_handler any time there is
// a priority ANT message in the queue
static void ant_rp_priority_queue_processing()
{
    uint8_t status_flag;

    priority_queue_t dequeued_record;

    // Pop record for processing
    ant_pop_record_from_queue(&dequeued_record);
    NRF_LOG_DEBUG("<%d> Page: 0x%02X, Tx: %d",__LINE__,dequeued_record.data_page, dequeued_record.tx_count);
    // Check the tx_count in record
    if ( (dequeued_record.tx_count - 1) > 0)
    {
        // decrement and store again
        dequeued_record.tx_count--;
        ant_load_record_in_queue(dequeued_record.priority,
                                 dequeued_record.payload,
                                 dequeued_record.data_page,
                                 dequeued_record.status,
                                 dequeued_record.tx_count);
    }
    // refresh list with sort list call
    ant_sort_priority_queue();

    // get the status flag setting
    status_flag = dequeued_record.status;

    // need to check the special case: Transmit until a successful acknowledge is received
    if ( (status_flag & ANT_ACKNOWLEDGE_TILL_PASS) == ANT_ACKNOWLEDGE_TILL_PASS)
    {
        // need to store a copy of this dequeued_record
        memcpy(&m_copy_of_record, &dequeued_record, sizeof(priority_queue_t));
    }

    // the data page is in its own field
    switch(dequeued_record.data_page)
    {
        case ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE:
            ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE);
            ant_rp_message_send(m_message_payload, status_flag);
            break;

        case ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE:
            ant_rp_prepare_next_data_page(m_message_payload, ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE);
            ant_rp_message_send(m_message_payload, status_flag);
            break;

        case ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS:
            // this code is the same for both tests
            memset(m_message_payload, ANT_RP_RESERVED_ONES, ANT_STANDARD_DATA_PAYLOAD_SIZE);

            // Set page number
            m_message_payload[0] = ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS;


            // set sequence number from dequeued record
            m_message_payload[1] = dequeued_record.payload[1];


            // Request from the Send Data Page Request Form
            if (dequeued_record.payload[0] == ANT_RP_DATA_PAGE_TRANS_REQUEST)
            {
                // check that OP Code was entered
                switch(dequeued_record.payload[3])
                {
                    case I2C_OP_CODE:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];          // OP CODE in payload location [3]
                        m_message_payload[7] = lidar_nvm_config_get_twis_mode();    // Page request with op-code, insert value from system
                        break;
                    case FACTORY_RESET_OP_CODE:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];  // OP CODE in payload location [3]
                        m_message_payload[7] = ANT_RP_RESERVED_ONES;        // Set to Invalid, don't track as this is a one shot command
                        break;
                    case DETECTION_SENSITIVITY:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];              // OP CODE in payload location [3]
                        m_message_payload[7] = lidar_nvm_get_detection_sensitivity();   // Page request with op-code, insert value from system
                        break;

                    case ACQUISTION_COUNT:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];          // OP CODE in payload location [3]
                        m_message_payload[7] = lidar_nvm_get_acquisition_count();   // Page request with op-code, insert value from system
                        break;

                    case QUICK_TERMINATION:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];              // OP CODE in payload location [3]
                        m_message_payload[7] = lidar_nvm_get_quick_termination_mode();  // Page request with op-code, insert value from system
                        break;

                    case HIGH_ACCURACY_OP_CODE:
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];              // OP CODE in payload location [3]
                        m_message_payload[7] = lidar_nvm_get_high_accuracy_count();    // Page request with op-code, insert value from system
                        break;

                    case NVM_MODE_OP_CODE:
                    {
                        bool nvm_state = lidar_nvm_get_access_mode();
                        m_message_payload[1] = m_sequence_number;
                        m_message_payload[6] = dequeued_record.payload[3];              // OP CODE in payload location [3]
                        if (nvm_state == true)
                        {
                            m_message_payload[7] = NVM_STORAGE_ENABLED;    // insert value from system
                        }
                        else
                        {
                            m_message_payload[7] = NVM_STORAGE_DISABLED;    // insert value from system
                        }
                        break;
                    }

                    case ANT_RP_INVALID_OP_CODE:
                    default:
                        NRF_LOG_INFO("Op code: %d has not been hooked into ANT module",dequeued_record.payload[3]);
                        break;
                }
            }
            else    // this is the Transmit Config Command Page Request 50
            {
                // check the OP Code that was entered, mirror payload transmit back for user feedback
                switch(dequeued_record.payload[6])
                {
                    case I2C_OP_CODE:
                    case FACTORY_RESET_OP_CODE:
                    case DETECTION_SENSITIVITY:
                    case ACQUISTION_COUNT:
                    case QUICK_TERMINATION:
                    case HIGH_ACCURACY_OP_CODE:
                    case NVM_MODE_OP_CODE:
                        m_message_payload[6] = dequeued_record.payload[6];  // OP CODE in payload location [6]
                        m_message_payload[7] = dequeued_record.payload[7];  // return value that was set on page
                        break;

                    case ANT_RP_INVALID_OP_CODE:
                    default:
                        NRF_LOG_INFO("Op code: %d has not been hooked into ANT module",dequeued_record.payload[6]);
                        break;
                }
            }

            // transmit message
            ant_rp_message_send(m_message_payload, status_flag);
            break;

        case ANT_RP_DATA_PAGE_POWER_MODE:
            // clear buffer
            memset(m_message_payload, ANT_RP_RESERVED_ONES, ANT_STANDARD_DATA_PAYLOAD_SIZE);

            // Set page number
            m_message_payload[0] = ANT_RP_DATA_PAGE_POWER_MODE;

            // set sequence number from dequeued record
            m_message_payload[1] = m_sequence_number;
            m_message_payload[4] = lidar_nvm_get_power_mode();
            m_message_payload[7] = lidar_nvm_get_measurement_interval();

            // transmit message
            ant_rp_message_send(m_message_payload, status_flag);
            break;

        case ANT_RP_DATA_PAGE_TRIGGER_MEASUREMENT:
            // clear buffer
            memset(m_message_payload, ANT_RP_RESERVED_ONES, ANT_STANDARD_DATA_PAYLOAD_SIZE);

            // Set page number
            m_message_payload[0] = ANT_RP_DATA_PAGE_TRIGGER_MEASUREMENT;
            // Set sequence number
            m_message_payload[1] = dequeued_record.payload[1];

            // transmit message
            ant_rp_message_send(m_message_payload, status_flag);

            break;

        default:
            NRF_LOG_INFO("Data page has not been hooked in: %d",dequeued_record.data_page);
            break;
    }
}

// insert a message that was not Acknowledged
// message is stored in static structure m_copy_of_record
static void ant_rp_insert_failed_message()
{
    ant_load_record_in_queue(ANT_HIGH_PRIORITY_MESSAGE,
                             m_copy_of_record.payload,
                             m_copy_of_record.data_page,
                             m_copy_of_record.status,
                             m_copy_of_record.tx_count);
    ant_sort_priority_queue();   // always sort after inserting records.
    memset(&m_copy_of_record, 0x00, sizeof( priority_queue_t));
}

// process page requests from the Display
static void ant_rp_process_rx_message(uint8_t *message)
{
    switch(message[0])
    {
        case ANT_RP_DATA_PAGE_TRANS_REQUEST:  // request data page
            switch(message[6])
            {
                case ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE:
                    ant_rp_store_dp_request_in_priority_queue(message, ANT_RP_DATA_PAGE_MANUFACTURER_ID_PAGE);
                    break;

                case ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE:
                    ant_rp_store_dp_request_in_priority_queue(message, ANT_RP_DATA_PAGE_PRODUCT_ID_PAGE);
                    break;

                // configuration READ OP_CODE request
                case ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS:
                    ant_rp_store_dp_request_in_priority_queue(message, ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS);
                    break;

                case ANT_RP_DATA_PAGE_POWER_MODE:
                    ant_rp_store_dp_request_in_priority_queue(message, ANT_RP_DATA_PAGE_POWER_MODE);
                    break;

                default:
                    NRF_LOG_INFO("Page request not Recognized: %d",message[6]);
                    break;
            }
            break;

        // configuration WRITE OP_CODE and Value
        case ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS:
            if (m_sequence_number != message[1])
            {
                // load record twice for configuration page, Team meeting feedback Oct 30th, 2018
                ant_load_record_in_queue(ANT_LOW_PRIORITY_MESSAGE, message, ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS, ANT_REGULAR_BROADCAST_MESSAGE,2);
                ant_sort_priority_queue();

                m_sequence_number = message[1];

                // this is to process a configuration command for an OP CODE
                ant_message_receive_configuration_setting(message, ANT_RP_DATA_PAGE_CONFIGURE_SETTINGS);
            }
            else
            {
                NRF_LOG_INFO("Duplicate LLV4 configure command detected");
            }
            break;

        // Trigger a measurement
        case ANT_RP_DATA_PAGE_TRIGGER_MEASUREMENT:
            if (m_sequence_number != message[1])
            {
                // store the sequence number for reference
                m_sequence_number = message[1];

                // Pass in "true" for (w/ bias correction)
                if (lidar_lite_request_measurement(true) == LIDAR_LITE_SUCCESS)
                {
                    measurement_started();
                }
            }
            else
            {
                NRF_LOG_INFO("Duplicate measurement command detected");
            }
            break;

        case ANT_RP_DATA_PAGE_POWER_MODE:
            NRF_LOG_INFO("Power mode configuration page received from Simulator");
            if (m_sequence_number != message[1])
            {
                // load record twice for power mode page, Team meeting feedback Oct 30th, 2018
                ant_load_record_in_queue(ANT_LOW_PRIORITY_MESSAGE, message, ANT_RP_DATA_PAGE_POWER_MODE, ANT_REGULAR_BROADCAST_MESSAGE,2);
                ant_sort_priority_queue();

                // store the sequence number for reference
                m_sequence_number = message[1];

                ant_message_receive_configuration_setting(message, ANT_RP_DATA_PAGE_POWER_MODE);
            }
            else
            {
                NRF_LOG_INFO("Duplicate Power Mode command detected");
            }
            break;

        default:
            NRF_LOG_INFO("Page received from Display needs addtional code to process: %d", message[0]);
            break;
    }
}

// function to process the retries, the acknowledgments and insert into priority queue
static void ant_rp_store_dp_request_in_priority_queue(uint8_t *message, uint8_t page)
{
    uint8_t i, number_of_retries, available_size;
    uint8_t status_flag = ANT_REGULAR_BROADCAST_MESSAGE;    // Regular broadcast message

    // extract the number of retries, 127 Maximum
    number_of_retries = message[5] & ANT_MAX_7_BIT_COUNT_MASK;

    // Check ACK bit
    if ((message[5] & ANT_MESSAGE_ACKNOWLEDGE_MASK) == ANT_MESSAGE_ACKNOWLEDGE_MASK)
    {
        if (number_of_retries == 0)
        {
            status_flag = ANT_ACKNOWLEDGE_TILL_PASS;
        }
        else
        {
            status_flag = ANT_ACKNOWLEDGE_REQUESTED;
        }
    }

    if (status_flag == ANT_ACKNOWLEDGE_TILL_PASS)
    {
        NRF_LOG_INFO("Transmit until successful acknowledge");
        // store one message once as this will keep getting reset as required
        ant_load_record_in_queue(ANT_HIGH_PRIORITY_MESSAGE, message, page, status_flag, 1);
        ant_sort_priority_queue();   // always sort after inserting records.
    }
    else // catch the other two cases, ACK with no retries or regular broadcast
    {
        if (number_of_retries != 0)
        {
            ant_load_record_in_queue(ANT_LOW_PRIORITY_MESSAGE, message, page, status_flag, number_of_retries);
            ant_sort_priority_queue();   // always sort after inserting records.

            if (status_flag == ANT_REGULAR_BROADCAST_MESSAGE)
            {
                NRF_LOG_INFO("Regular Broadcast. Message count: %d",number_of_retries);
            }
            else
            {
                NRF_LOG_INFO("Transmit until successful not requested. Message count: %d",number_of_retries);
            }
        }
        else
        {
            NRF_LOG_INFO("Requested transmission response - INVALID");
        }
    }

}

// called from ant_message_parser
void ant_rp_set_distance_error()
{
    m_last_measured_distance_cm = ANT_RP_MEASUREMENT_FAILED;
}

// Method to reset power level back to ALWAYS_ON so that the toggling
// of the FPGA in the library doesn't get us into an undesirable state
void ant_rp_power_mode_reset()
{
    m_ANT_power_mode = LIDAR_LITE_ALWAYS_ON;
}

// A new interval measurement has been received over ANT, I2C, from power up
// interval = 0, do not skip any channel periods
// interval = 1, skip one channel period
// interval = 2, skip two channel periods
// etc
void ant_rp_measurement_interval_udpate(uint8_t interval)
{
    m_distance_measurement_counter = 0; // reset the counter
    if (interval == 0x00 || interval == ANT_RP_RESERVED_ONES)
    {
        // force algorithm to send measurement every channel period if ENABLED
        m_distance_measurement_interval = ANT_RP_RESERVED_ONES;
    }
    else
    {
        // add ONE so that number of channel periods skipped is correct
        m_distance_measurement_interval = interval + 1;

        // check for MAX size as 0xFF is our one byte boundary.
        if (m_distance_measurement_interval >= ANT_RP_RESERVED_ONES - 1)
        {
            // MAX is 254, skip 253 channel periods
            m_distance_measurement_interval = ANT_RP_RESERVED_ONES - 1;
        }
    }
}


// Method to update local variable with NEW power level
// The new power level is received over an ANT message or a virtual I2C register or from power up
void ant_rp_power_mode_update(lidar_lite_power_config_t power_mode)
{
    NRF_LOG_INFO("Updating power level variable ANT module to: 0x%02X",power_mode);
    m_ANT_power_mode = power_mode;
}

// the functions below here are used to implement the priority queue system for ANT messages
static void ant_load_record_in_queue(uint8_t priority, uint8_t *payload, uint8_t pagenumber, uint8_t status, uint8_t copies)
{
    int8_t record = ant_get_empty_record_from_queue();
    if (record >= 0)
    {
        m_ant_priority_queue[record].priority = priority;
        m_ant_priority_queue[record].data_page = pagenumber;
        m_ant_priority_queue[record].status = status;
        m_ant_priority_queue[record].tx_count = copies;
        memcpy(m_ant_priority_queue[record].payload, payload, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    }
    else
    {
        NRF_LOG_DEBUG("No space in priority queue");
    }

}

static int8_t ant_get_empty_record_from_queue()
{
    int8_t i;
    for (i=0; i< PRIORITY_QUEUE_SIZE; i++)
    {
        if (m_ant_priority_queue[i].priority == 0)
        {
            return i;
        }
    }
    return -1;
}

// since the queue is always sorted after records are added or popped
// this is the optimized way of returning size
static int8_t ant_get_priority_queue_size()
{
    int8_t i = 0;

    while (m_ant_priority_queue[i].priority != 0)
    {
        i++;
    }
    return i;
}

static void ant_pop_record_from_queue(priority_queue_t *record)
{
    int8_t rec = ant_get_priority_queue_size();
    if (rec != 0)
    {
        // get the first record from the queue
        memcpy(record, &m_ant_priority_queue[0], sizeof *record);
        // zero out popped record
        memset(&m_ant_priority_queue[0], 0x00, sizeof(m_ant_priority_queue[0]));
    }
    else
    {
        NRF_LOG_DEBUG("No Records in priority queue");
    }
}

static void ant_sort_priority_queue()
{
    qsort(m_ant_priority_queue, PRIORITY_QUEUE_SIZE, sizeof(priority_queue_t), ant_sort_compare_function);
}

// comparison function signature must return an int, otherwise warning is generated.
static int ant_sort_compare_function(const void *a, const void *b)
{
    priority_queue_t *x = (priority_queue_t*) a;
    priority_queue_t *y = (priority_queue_t*) b;
    // reverse order sort, highest priority on top
    return y->priority - x->priority;
}

// flush priority_queue
static void ant_flush_priority_queue()
{
    memset(&m_ant_priority_queue[0], 0x00, sizeof(m_ant_priority_queue));
}