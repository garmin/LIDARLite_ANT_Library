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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "app_scheduler.h"

#include "lidar_lite_interface.h"
#include "serial_twis.h"
#include "serial_twis_parser.h"
#include "lidar_config_nvm.h"
#include "sdk_config.h"
#include "fds.h"

#define NRF_LOG_MODULE_NAME ll_serial_twis
#if NRF_LOG_ENABLED && LL_TWI_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       LL_TWI_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL       0
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

//Local functions
static void serial_twis_write_request(void);
static void serial_twis_write_complete(size_t cnt);
static void serial_twis_read_request(void);
static void serial_twis_read_complete(size_t cnt);
static uint32_t serial_twis_id_number_maker(uint8_t user_input[]);
static ret_code_t lidar_lite_serial_twis_restart();
static ret_code_t enable_i2c_addresses(uint8_t primary, uint8_t secondary);

static uint8_t m_rxbuff[LL_TWIS_RX_BUFFER_SIZE];    // Receive buffer for data coming from TWI interface.

static uint8_t m_txbuff[LL_TWIS_TX_BUFFER_SIZE];    // Transmit buffer for data coming from TWI interface.
static uint32_t m_txbuff_used = 0;                  // Represents how much usable data was placed into the transmit buffer
// variables and functions used in packing bytes into I2C buffer
#define MAX_LIBRARY_REG_SIZE    256
#define WIRE_LIB_BUFFER_SIZE    32                  // Match the buffer size used in the Arduino
static uint8_t register_start_address       = 0x00;
static uint8_t library_register_values[MAX_LIBRARY_REG_SIZE] = {0}; // Library FPGA register values
static uint16_t offset, start, wrap_around;                         // assign static global for speed up

static const nrf_drv_twis_t m_twis = NRF_DRV_TWIS_INSTANCE(LL_TWIS_INST);     // TWIS driver instance.

// A copy of the event handler must be saved for when a user sets a Second slave address
static nrf_drv_twis_event_handler_t m_copy_event_handler;
static uint8_t m_secondary_i2c_slave_address    = LL_I2C_DEACTIVATED;   // secondary i2c address always defaulted to zero on startup
static uint8_t m_primary_i2c_address            = LL_TWIS_SLAVE_ADDR;   // created to support I2C_CONFIG
static ll_unit_id_t m_unit_identification;                              // created to support slicing of ANT_ID

// Write request, the TWI Master (External MCU) is about to send data, prepare the receive buffer
static void serial_twis_write_request(void)
{
    uint32_t err_code;
    err_code = nrf_drv_twis_rx_prepare(&m_twis, m_rxbuff, sizeof(m_rxbuff));
    APP_ERROR_CHECK(err_code);
}

// Write request completed, the TWI master (External MCU) has finished sending data, decode the message
// Note: size_t only consists of the 'payload' and does not factor in the byte transmitted for the twi slave address (0x62)
static void serial_twis_write_complete(size_t cnt)
{
    bool is_register_read_request = false;

    // Specific to the Lidar Lite I2C protocol
    // If the write request only contains the register address, then the external MCU will perform a TWI read on that register address
    // If the write request contains the register address and additional data, then the MCU is attempting to write to that address
    if (cnt == LL_TWIS_REGISTER_ADDR_SIZE)
    {
        is_register_read_request = true;
    }

    // Starting address for multi-byte reads, located in m_rxbuff[0]
    register_start_address = m_rxbuff[0];

    //Decode received message
    serial_twis_parser_decode_message(m_rxbuff, cnt, is_register_read_request);
}

// Read request, the TWI master (External MCU) has requested data from the TWI slave, load the transmit buffer (that was prepared off of a write request)
static void serial_twis_read_request(void)
{
    switch (register_start_address)
    {
        // Correlation record is read TWO bytes at a time.
        case LL_REGISTER_CORR_DATA:
            break;

        default:
        {
            // copy library register values
            lidar_lite_get_library_register_cache(library_register_values);
            start = register_start_address;

            wrap_around = start + WIRE_LIB_BUFFER_SIZE;

            if (wrap_around > MAX_LIBRARY_REG_SIZE - 1)
            {
                offset = abs(MAX_LIBRARY_REG_SIZE - wrap_around);
                // copy first part of buffer
                memcpy(m_txbuff, &library_register_values[start], MAX_LIBRARY_REG_SIZE - start);
                // copy the wrap around
                memcpy(&m_txbuff[WIRE_LIB_BUFFER_SIZE - offset], &library_register_values[0], offset);
            }
            else
            {
                memcpy(m_txbuff, &library_register_values[start], WIRE_LIB_BUFFER_SIZE);
            }

            m_txbuff_used = WIRE_LIB_BUFFER_SIZE;
        }
        break;
    }
    (void) nrf_drv_twis_tx_prepare(&m_twis, m_txbuff, m_txbuff_used);
}

// Read request completed (TWI slave transmit), verify that the entire transmit buffer was sent.
static void serial_twis_read_complete(size_t cnt)
{
    // Verify TWI interface sent the expected amount of bytes from the transmit buffer (if unequal, master requested a different amount)
    if (m_txbuff_used != cnt)
    {
        NRF_LOG_WARNING("Tx buffer had size %d and TWI interface sent %d bytes", m_txbuff_used, cnt);
    }
}

void serial_twis_prepare_tx_buffer(uint8_t * p_tx_data, uint32_t tx_data_size)
{
    APP_ERROR_CHECK_BOOL(tx_data_size <= LL_TWIS_TX_BUFFER_SIZE);

    // Monitor the current size of the used tx buffer, used for error checking
    m_txbuff_used = tx_data_size;

    // Prepare the transmit buffer for a TWIS read
    for(int i = 0; i < tx_data_size; i++)
    {
        m_txbuff[i] = p_tx_data[i];
    }
}


void lidar_lite_serial_twis_event_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code;
    nrf_drv_twis_evt_t * p_event = (nrf_drv_twis_evt_t*) p_event_data;

    switch (p_event->type)
    {
        case TWIS_EVT_READ_REQ:
            if (p_event->data.buf_req)
            {
                NRF_LOG_DEBUG("Read Request");
                serial_twis_read_request();
            }
            break;
        case TWIS_EVT_READ_DONE:
            NRF_LOG_DEBUG("Read Done");
            serial_twis_read_complete(p_event->data.tx_amount);
            break;
        case TWIS_EVT_WRITE_REQ:
            if (p_event->data.buf_req)
            {
                NRF_LOG_DEBUG("Write Request");
                serial_twis_write_request();
            }
            break;
        case TWIS_EVT_WRITE_DONE:
            NRF_LOG_DEBUG("Write Done");
            serial_twis_write_complete(p_event->data.rx_amount);
            break;

        case TWIS_EVT_READ_ERROR:
            err_code = nrf_drv_twis_error_get_and_clear(&m_twis);
            NRF_LOG_ERROR("TWIS READ ERROR... Code: %d", err_code);
            break;
        case TWIS_EVT_WRITE_ERROR:
            err_code = nrf_drv_twis_error_get_and_clear(&m_twis);
            NRF_LOG_ERROR("TWIS WRITE ERROR... Code: %d", err_code);
            break;
        case TWIS_EVT_GENERAL_ERROR:
            err_code = nrf_drv_twis_error_get_and_clear(&m_twis);
            NRF_LOG_ERROR("TWIS GENERAL ERROR... Code: %d", err_code);
            break;
        default:
            break;
    }
}


void lidar_lite_serial_twis_event_schedule(nrf_drv_twis_evt_t const * const p_event)
{
    app_sched_event_put((void*)p_event, sizeof(nrf_drv_twis_evt_t), lidar_lite_serial_twis_event_handler);
}


ret_code_t lidar_lite_serial_twis_init(nrf_drv_twis_event_handler_t event_handler)
{
    ret_code_t ret;

    const nrf_drv_twis_config_t config =
    {
        .addr               = {m_primary_i2c_address, m_secondary_i2c_slave_address},
        .scl                = LL_TWIS_SCL_PIN,
        .scl_pull           = LL_TWIS_SCL_PIN_PULL,
        .sda                = LL_TWIS_SDA_PIN,
        .sda_pull           = LL_TWIS_SDA_PIN_PULL,
        .interrupt_priority = TWIS_DEFAULT_CONFIG_IRQ_PRIORITY
    };

    // create copy of event_handler for restarts.
    m_copy_event_handler = event_handler;
    // store the ANT_ID for later use
    m_unit_identification.shared.ant_id = LL_ANT_ID;
    // store the ANT_ID in library register cache
    lidar_lite_set_library_register_value(LL_REGISTER_UNIT_ID_0, m_unit_identification.shared.parts[0]);
    lidar_lite_set_library_register_value(LL_REGISTER_UNIT_ID_1, m_unit_identification.shared.parts[1]);
    lidar_lite_set_library_register_value(LL_REGISTER_UNIT_ID_2, m_unit_identification.shared.parts[2]);
    lidar_lite_set_library_register_value(LL_REGISTER_UNIT_ID_3, m_unit_identification.shared.parts[3]);

    // Init TWIS
    do
    {
        ret = nrf_drv_twis_init(&m_twis, &config, event_handler);
        if (NRF_SUCCESS != ret)
        {
            break;
        }

        nrf_drv_twis_enable(&m_twis);
    }while (0);
    return ret;
}

// Restart the Two wire interface when there are slave address changes/additions
static ret_code_t serial_twis_restart()
{
    ret_code_t err_code;

    const nrf_drv_twis_config_t config =
    {
        .addr               = {m_primary_i2c_address, m_secondary_i2c_slave_address},
        .scl                = LL_TWIS_SCL_PIN,
        .scl_pull           = LL_TWIS_SCL_PIN_PULL,
        .sda                = LL_TWIS_SDA_PIN,
        .sda_pull           = LL_TWIS_SDA_PIN_PULL,
        .interrupt_priority = TWIS_DEFAULT_CONFIG_IRQ_PRIORITY
    };

    // Init TWIS
    do
    {
        err_code = nrf_drv_twis_init(&m_twis, &config, m_copy_event_handler);
        if (NRF_SUCCESS != err_code)
        {
            break;
        }
        nrf_drv_twis_enable(&m_twis);
    }while (0);

    return err_code;
}


// set the secondary i2c address received from API call
ret_code_t serial_twis_set_secondary_i2c_address(uint8_t i2c_address)
{
    ret_code_t err_code;

    // store new address in static variable
    m_secondary_i2c_slave_address = i2c_address;

    // disable the TWI for slave, if this is not called APP_ERROR_CHECK()
    // will halt the system as the TWI cannot be restarted as it is still active
    // void function no return checks
    serial_twis_disable();

    // Restart TWI as it is not active
    err_code = serial_twis_restart();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

// buffer is exactly five bytes long
// buffer[0] = ANT_ID[ byte one ] LSB
// buffer[1] = ANT_ID[ byte two ]
// buffer[2] = ANT_ID[ byte three ]
// buffer[3] = ANT_ID[ byte four ] MSB
// buffer[4] = New secondary i2c slave address
ret_code_t serial_twis_check_and_configure_secondary_i2c_address(uint8_t *buffer)
{
    ret_code_t err_code;
    ASSERT(buffer != NULL);

    uint8_t i2c_address = (uint8_t)buffer[4];   // store i2c_slave address, location [4]
    uint32_t user_input = serial_twis_id_number_maker(buffer);  // make serial number from buffer

    if (m_unit_identification.shared.ant_id == user_input)
    {
        // the correct ANT_ID was received and matches our UICR register
        // check that the i2c_address is 7bits only
        if (i2c_address <= MAX_I2C_SLAVE_ADDRESS)
        {
            // store new address in NVM
            err_code = lidar_nvm_config_set_slave_address(i2c_address);
            if (err_code != FDS_SUCCESS)
            {
                NRF_LOG_ERROR("Secondary slave address could not be written to NVM");
            }
            lidar_lite_set_library_register_value(LL_REGISTER_I2C_SEC_ADDR, i2c_address);
            // All call are checking NRF_SUCCESS which is equal to FDS_SUCCESS
            return err_code;
        }
        else
        {
            // Secondary slave address passed in from I2C interface command is invalid
            // Do nothing return error
            NRF_LOG_ERROR("Invalid I2C secondary slave address");
            return NRF_ERROR_INVALID_PARAM;
        }
    }
    else
    {
        // ANT_ID did not match, do nothing, return error
        NRF_LOG_ERROR("ANT/UNIT ID did not match");
        return NRF_ERROR_INVALID_PARAM;
    }

}

// configure which i2c addresses to use
static ret_code_t enable_i2c_addresses(uint8_t primary, uint8_t secondary)
{
    ret_code_t  err_code;

    m_primary_i2c_address           = primary;
    m_secondary_i2c_slave_address   = secondary;

    // disable the TWI for slave, if this is not called APP_ERROR_CHECK()
    // will halt the system as the TWI cannot be restarted as it is still active
    // void function no return checks
    serial_twis_disable();

    // Restart TWI as it is not active
    err_code = serial_twis_restart();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

ret_code_t serial_twis_configure_i2c_addresses(uint8_t mode)
{
    ret_code_t err_code = NRF_SUCCESS;

    uint8_t i2c_address, primary, secondary;

    // store I2C mode in library cache
    lidar_lite_set_library_register_value(LL_REGISTER_I2C_CONFIG, mode);

    switch (mode)
    {
        case LL_DEFAULT_ADDR_ONLY:
            {
                primary     = LL_TWIS_SLAVE_ADDR;
                secondary   = LL_I2C_DEACTIVATED;

                NRF_LOG_INFO("Default I2C address enabled, Secondary disabled");
                // if this fails, the APP_ERROR_CHECK will halt system
                return enable_i2c_addresses(primary, secondary);
            }
            break;
        case LL_SECONDARY_ADDR_ONLY:
            {
                // retrieve secondary address
                err_code = lidar_nvm_config_get_slave_address(&i2c_address);
                if (err_code == FDS_SUCCESS)        // NRF_SUCCESS == FDS_SUCCESS
                {
                    if (i2c_address != LL_I2C_DEACTIVATED)
                    {
                        primary     = LL_I2C_DEACTIVATED;     // disable default address
                        secondary   = i2c_address;

                        // if this fails, the APP_ERROR_CHECK will halt the system
                        err_code = enable_i2c_addresses(primary, secondary);
                    }
                    else
                    {
                        // cant set, as ZERO will disable interface
                        NRF_LOG_ERROR("Secondary slave address == 0x00, Can not set as this will disable TWI on LLV4");
                        err_code = NRF_ERROR_INVALID_DATA;
                    }
                }
                else
                {
                    // secondary address could not be retrieved from NVM
                    // force slave to, 0x00, restart TWI interface with default primary address 0x62

                    primary = LL_TWIS_SLAVE_ADDR;
                    secondary = LL_I2C_DEACTIVATED;
                    NRF_LOG_INFO("Default I2C address set, Secondary deactivated as read from NVM was not successful");
                    // if this fails, the APP_ERROR_CHECK will halt the system
                    err_code = enable_i2c_addresses(primary, secondary);
                }

                return err_code;
            }
            break;

        case LL_BOTH_ADDRESSES:
            {
                // enable both addresses, or at a minimum enable default address
                // read secondary slave address
                err_code = lidar_nvm_config_get_slave_address(&i2c_address);
                if (err_code == FDS_SUCCESS)
                {
                    primary = LL_TWIS_SLAVE_ADDR;
                    secondary = i2c_address;

                    NRF_LOG_INFO("Primary and Secondary I2C addresses are set");
                    // if this fails, the APP_ERROR_CHECK will halt the system
                    err_code = enable_i2c_addresses(primary, secondary);
                }
                else
                {
                    // ensure that the default address is enabled
                    primary = LL_TWIS_SLAVE_ADDR;
                    secondary = LL_I2C_DEACTIVATED;
                    NRF_LOG_INFO("Default I2C address set, Secondary deactivated as read from NVM was not successful");
                    // if this fails, the APP_ERROR_CHECK will halt the system
                    err_code = enable_i2c_addresses(primary, secondary);
                }
                return err_code;
            }
            break;

        default:
            NRF_LOG_ERROR("Invalid parameter received on serial_twis_configure_i2c_addresses");
            return NRF_ERROR_INVALID_PARAM;
            break;
    }

}

// Getter function for Unit ID from structure ll_unit_id_t
void serial_twis_get_unit_identification(uint8_t *buffer)
{
    // shared is a union in m_unit_identification
    // the LLV4's ANT_ID is stored at power up as a uint32_t
    // uint8_t parts[4]
    buffer[0] = m_unit_identification.shared.parts[0];
    buffer[1] = m_unit_identification.shared.parts[1];
    buffer[2] = m_unit_identification.shared.parts[2];
    buffer[3] = m_unit_identification.shared.parts[3];
}

// shutdown I2C interface
void serial_twis_disable()
{
    // these function do not return anything
    nrf_drv_twis_disable(&m_twis);
    nrf_drv_twis_uninit(&m_twis);
}

static uint32_t serial_twis_id_number_maker(uint8_t user_input[])
{
    // convert input from first 4 bytes of user_input[]
    // use the union in ll_unit_id_t to convert 4 bytes to a uint32_t value
    ll_unit_id_t user_id;

    user_id.shared.parts[0] = user_input[0];
    user_id.shared.parts[1] = user_input[1];
    user_id.shared.parts[2] = user_input[2];
    user_id.shared.parts[3] = user_input[3];

    return user_id.shared.ant_id;
}
