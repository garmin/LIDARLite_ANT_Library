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

#ifndef __SERIAL_TWIS__
#define __SERIAL_TWIS__

#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_drv_twis.h"
#include "app_interface.h"

// Lidar Lite TWI pin pull configuration
#define LL_TWIS_SCL_PIN_PULL              NRF_GPIO_PIN_PULLUP
#define LL_TWIS_SDA_PIN_PULL              NRF_GPIO_PIN_PULLUP

#define LL_TWIS_RX_BUFFER_SIZE            255    // The maximum number of bytes that can be received over the I2C interface (Can not be more than 255!)
#define LL_TWIS_TX_BUFFER_SIZE            255    // The maximum number of bytes that can be transfered over the I2C interface (Can not be more than 255!)

#define LL_TWIS_SLAVE_ADDR                0x62    // TWI slave address.
#define LL_TWIS_REGISTER_ADDR_SIZE        1       // Number of bytes used to address FPGA registers

#define LL_TWIS_SCL_PIN                   NRF_GPIO_PIN_MAP(0,7)
#define LL_TWIS_SDA_PIN                   NRF_GPIO_PIN_MAP(0,6)

#define LL_TWIS_INST                      1       // TWIS interface instance, 0 is currently used by the SPI interface

// Location of ANT_ID
#define LL_ANT_ID                         NRF_UICR->CUSTOMER[SYSTEM_UICR_CUST_ANT_ID_OFFSET]

// structure to store the LL_ANT_ID and retrieve the bytes together
// or one at a time.  little Endian order
typedef struct
{
    union
    {
        uint32_t ant_id;        // 0x01020304
        uint8_t parts[4];       // [0] = 04, [1] = 03, [2] = 02, [3] = 01
    }shared;
}ll_unit_id_t;

// This is the maximum value the I2C slave address can be set at
#define MAX_I2C_SLAVE_ADDRESS             0x7F

// definitions for I2C_CONFIG option
#define LL_DEFAULT_ADDR_ONLY              0x00
#define LL_SECONDARY_ADDR_ONLY            0x01
#define LL_BOTH_ADDRESSES                 0x02
#define LL_I2C_DEACTIVATED                0x00

/**
 * \defgroup twis_handlers TWI event handlers.
 * @ details Functions in this group can be used to enable I2C (TWI) communication with an external MCU
 * @{
 */

/** @brief Lidar Lite twis initialization
 *
 * @details The twis event handler is exposed to the application so that it can handle the scheduling of incoming events.
 *          The twis event should be passed back into @ref lidar_lite_twis_event_handler so that they can be properly processed.
 *          This function must be called explicitly by the application in order to enable I2C communication.
 *
 * @param[in] event_handler   TWI slave event callback function type.
 */
ret_code_t lidar_lite_serial_twis_init(nrf_drv_twis_event_handler_t event_handler);

/** @brief Handle twis events
 *
 * @details Application should pass incoming twis events into this function so they may be processed by the Lidar Lite library.
 *
 * @param[in] p_event_data    Event data (use a pointer to @ref nrf_drv_twis_event_t)
 * @param[in] event_size      Size of event data
 */
void lidar_lite_serial_twis_event_handler(void * p_event_data, uint16_t event_size);

/**@brief   Handle twis events using the scheduler
 *
 * @details This function will handle any twis events from an external MCU and place them into the scheduler,
 *          the scheduler will pass this event back to the Lidar Lite library once it is ready for processing.
 */
void lidar_lite_serial_twis_event_schedule(nrf_drv_twis_evt_t const * const p_event);

/**@}*/

/**
* This function is used to prepare the transmit buffer and must be called before a read request
* The provided transmit buffer will be sent once the TWI Master (External MCU) performs a read request
 */
void serial_twis_prepare_tx_buffer(uint8_t * p_tx_data, uint32_t tx_data_size);

 /** @brief     API call to check and set secondary slave address
 *              This routine checks the ANT_ID for a match before setting the secondary address
 *
 * @param[in]   buffer, contains five bytes, ANT_ID (4 bytes) + Address (1 byte)
 *
 * @retval      NRF_SUCCESS
 *              NRF_ERROR_INVALID_DATA
 *              NRF_ERROR_INVALID_PARAM
 */
ret_code_t serial_twis_check_and_configure_secondary_i2c_address(uint8_t *buffer);

 /** @brief     Configure both interfaces on TWI
 *
 * @param[in]   mode:
 *              0x00: Enable Default I2C address (0x62)
 *              0x01: Enable secondary I2c address if set
 *              0x02: Both addresses are enabled
 *
 * @retval      NRF_SUCCESS
 *              NRF_ERROR_INVALID_DATA
 *              NRF_ERROR_INVALID_PARAM
 */
ret_code_t serial_twis_configure_i2c_addresses(uint8_t mode);

/** @brief     Returns the UNIT_ID in buffer
 *
 * @param[out]   buffer:  size is allocated in caller, currently 4 bytes
 *
 * @retval      None
*/
void serial_twis_get_unit_identification(uint8_t *buffer);

/** @brief     API call to set the secondary I2C slave address in fqke NVM store
 *
 * @param[in]   i2c_address
 *
 * @retval      NRF_SUCCESS
*/
ret_code_t serial_twis_set_secondary_i2c_address(uint8_t i2c_address);

/** @brief     Disable both I2C addresses
 *
 * @retval      None, nrf Library calls do not return succcess or failure
*/
void serial_twis_disable();
#endif
