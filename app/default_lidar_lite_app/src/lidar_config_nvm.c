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
#include "sdk_errors.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "fds.h"
#include "lidar_config_nvm.h"
#include "serial_twis.h"
#include "serial_twis_parser.h"
#include "lidar_lite_defines.h"
#include "ant_message_parser.h"
#include "lidar_lite_interface.h"

#define NRF_LOG_MODULE_NAME lidar_config_nvm
#if NRF_LOG_ENABLED && LL_NVM_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       LL_NVM_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL       0
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

static void lidar_config_nvm_evt_handler(void * p_evt, uint16_t event_size);        // Local FDS event handler
static void lidar_config_nvm_general_handler(void *p_event_data, uint16_t size);    // local events are handled here
static lidar_nvm_event_handler_t m_lidar_nvm_callback_handler;                      // callback handler to main
static void lidar_lite_install_second_address();
static void lidar_config_nvm_event_scheduler(fds_evt_t const * p_event);

/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

static lidar_config_nvm_state_t                 m_lidar_config_nvm_state;
static lidar_fpga_config_t                      m_temp_lidar_fpga_cfg;
static ll_nvm_evt_t                             m_nvm_event_type;
static bool m_slave_address_update_pending      = false;
static bool m_fds_copy_in_memory                = false;
static bool m_pending_factory_reset             = false;
static bool m_nvm_busy                          = false;

/* Lidar FPGA configuration data. */
static lidar_fpga_config_t m_lidar_fpga_cfg =
{
    .HEAD                       = {0xDE, 0xEE, 0xED},               // LLV4 configuration start
    .enable_nvm_storage         = DEFAULT_DISABLE_NVM_STORAGE,
    .secondary_slave_address    = DEFAULT_SECONDARY_I2C_ADDR,
    .i2c_enabled                = DEFAULT_I2C_CONFIGURATION,
    .power_mode                 = DEFAULT_POWER_MODE,
    .measurement_interval       = DEFAULT_MEASUREMENT_INTERVAL,
    .detection_sensitivity      = DEFAULT_DETECTION_SENSITIVITY,
    .acquisition_count          = DEFAULT_ACQUISITION_COUNT,
    .quick_termination          = DEFAULT_QUICK_TERMINATION,
    .high_accuracy_count        = DEFAULT_HIGH_ACCURACY_COUNT,
    .TAIL                       = {0xDE, 0xEE, 0xED},               // LLV4 configuration end
};

/* A record containing Lidar Configuration configuration data. */
static fds_record_t const m_lidar_fpga_cfg_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &m_lidar_fpga_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_lidar_fpga_cfg) + 3) / sizeof(uint32_t),
};

ret_code_t lidar_config_nvm_init(lidar_nvm_event_handler_t event_handler)
{
    ret_code_t err_code;
    m_lidar_config_nvm_state = initializing;
    m_fds_copy_in_memory = false;

    // store callback handler
    m_lidar_nvm_callback_handler = event_handler;

    /* Register first to receive an event when initialization is complete. */
    // install so that FDS events are scheduled
    err_code = fds_register(lidar_config_nvm_event_scheduler);
    if (err_code != FDS_SUCCESS)
    {
        NRF_LOG_ERROR("NVM callback handler did not register. %s\r\n",fds_err_str[err_code]);
    }
    else
    {
        err_code = fds_init();
        if (err_code != FDS_SUCCESS)
        {
            NRF_LOG_ERROR("NVM initialization failure. %s\r\n",fds_err_str[err_code]);
        }
    }

    return err_code;
}

ret_code_t lidar_config_nvm_update(lidar_fpga_config_t const * updated_config)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  ftok = {0};

    NRF_LOG_INFO("<%d> NVM Update request",__LINE__);
    if (m_lidar_config_nvm_state == initialized)
    {
        // reset because we are updating fds
        m_fds_copy_in_memory = false;
        err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &ftok);
        if (err_code == FDS_SUCCESS)
        {
            fds_record_t const rec =
            {
                .file_id           = CONFIG_FILE,
                .key               = CONFIG_REC_KEY,
                .data.p_data       = updated_config,
                .data.length_words = (sizeof(lidar_fpga_config_t) + 3) / sizeof(uint32_t)
            };

            // Attempt at most two high level command writes to NVM
            // Internal counter is in sdk_config.h:  NRF_FSTORAGE_SD_MAX_RETRIES 8
            err_code = fds_record_update(&desc, &rec);
            if (err_code != FDS_SUCCESS)
            {
                NRF_LOG_ERROR("<%d> Error: FDS update failed on first attempt. %s\r\n", __LINE__,fds_err_str[err_code]);
                if (strstr("FDS_ERR_BUSY", fds_err_str[err_code]))
                {
                    // FDS was busy retry, re-use err_code
                    err_code = fds_record_update(&desc, &rec);
                    if (err_code != FDS_SUCCESS)
                    {
                        NRF_LOG_ERROR("<%d> Error: FDS update failed on second attempt. %s\r\n", __LINE__,fds_err_str[err_code]);
                    }
                }
            }
            // All sequences checked here
            if (err_code == FDS_SUCCESS)
            {
                m_fds_copy_in_memory = true;
            }
            else
            {
                // Notify the main application that the NVM failed on update attempts.
                ll_nvm_evt_t nvm_event;
                nvm_event.type = NVM_UPDATE_FAILURE;
                nvm_event.response_code = NRF_ERROR_INTERNAL;
                m_lidar_nvm_callback_handler(&nvm_event);
            }
        }
        else
        {
          NRF_LOG_ERROR("<%d> Could not find LL Config in NVM. Update error!. %s\r\n", __LINE__,fds_err_str[err_code]);
        }
    }
    else
    {
        NRF_LOG_ERROR("<%d> Lidar Config NVM Update called before FDS has been initialized.\r\n",__LINE__);
        err_code = FDS_ERR_NOT_INITIALIZED;
    }

    // one exit point only
    return err_code;
}

ret_code_t lidar_config_nvm_read(lidar_fpga_config_t* lidar_fpga_config)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    NRF_LOG_INFO("<%d> NVM Read request",__LINE__);
    if (m_lidar_config_nvm_state == initialized)
    {
        err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
        if (err_code == FDS_SUCCESS)
        {
            /* A config file is in flash. Let's update it. */
            fds_flash_record_t config = {0};

            /* Open the record and read its contents. */
            err_code = fds_record_open(&desc, &config);
            APP_ERROR_CHECK(err_code);

            /* Copy the configuration from flash into read_config. */
            memcpy(lidar_fpga_config, config.p_data, sizeof(lidar_fpga_config_t));

            /* Close the record when done reading. */
            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);

            m_fds_copy_in_memory = true;
        }
        else
        {
          NRF_LOG_ERROR("<%d> Lidar Config File Not Found. %s\r\n", __LINE__,fds_err_str[err_code]);
        }
    }
    else
    {
        NRF_LOG_ERROR("<%d> Lidar Config NVM Read called before FDS has been initialized. %d",__LINE__);
        err_code = FDS_ERR_NOT_INITIALIZED;
    }

    // one exit point only
    return err_code;
}

static void lidar_config_nvm_event_scheduler(fds_evt_t const * p_event)
{
    app_sched_event_put((void*)p_event, sizeof(fds_evt_t), lidar_config_nvm_evt_handler);
}

static void lidar_config_nvm_evt_handler(void * p_event_data, uint16_t event_size)
{
    fds_evt_t * p_evt = (fds_evt_t*) p_event_data;
    NRF_LOG_INFO("Event: %s received (%s)",
                  fds_evt_str[p_evt->id],
                  fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                if (m_lidar_config_nvm_state == initializing)
                {
                    uint32_t err_code;
                    m_lidar_config_nvm_state = initialized;

                    m_fds_copy_in_memory = false;

                    fds_stat_t stat = {0};

                    err_code = fds_stat(&stat);
                    APP_ERROR_CHECK(err_code);

                    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
                    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

                    fds_record_desc_t desc = {0};
                    fds_find_token_t  tok  = {0};

                    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
                    if (err_code == FDS_SUCCESS)
                    {
                        /* A config file is in flash. Let's update it. */
                        fds_flash_record_t config = {0};

                        /* Open the record and read its contents. */
                        err_code = fds_record_open(&desc, &config);
                        APP_ERROR_CHECK(err_code);

                        lidar_fpga_config_t lidar_fpga_config = *(lidar_fpga_config_t *)(config.p_data);

                        /* Check if NVM is enabled. Load the NVM to cache if yes, otherwise use default values*/
                        if(lidar_fpga_config.enable_nvm_storage == NVM_STORAGE_ENABLED)
                        {
                            /* Copy the configuration from flash into m_lidar_fpga_cfg. */
                            memcpy(&m_lidar_fpga_cfg, config.p_data, sizeof(lidar_fpga_config_t));
                        }

                        /* Maintain the secondary I2C address regardless NVM being enabled or not */
                        m_lidar_fpga_cfg.secondary_slave_address = lidar_fpga_config.secondary_slave_address;

                        /* Close the record when done reading. */
                        err_code = fds_record_close(&desc);
                        APP_ERROR_CHECK(err_code);

                    }
                    else
                    {
                        /* System config not found; write a new one. */
                        // this is the blank file structure, no valid data from LLv4
                        NRF_LOG_INFO("Writing config file...");

                        err_code = fds_record_write(&desc, &m_lidar_fpga_cfg_record);
                        APP_ERROR_CHECK(err_code);
                    }

                    m_fds_copy_in_memory = true;

                    // Notify the main application that the NVM is ready.
                    ll_nvm_evt_t nvm_event;
                    nvm_event.type = NVM_INIT_COMPLETE;
                    nvm_event.response_code = NRF_SUCCESS;
                    m_lidar_nvm_callback_handler(&nvm_event);

                    // schedule a garbage collection if required
                    // this may need to be adjusted when more items are being stored in the NVM
                    if (stat.dirty_records > 0)
                    {
                        // allocate event
                        m_nvm_event_type.type = NVM_GARBAGE_COLLECTION;
                        ll_nvm_evt_t * p_event = &m_nvm_event_type;
                        // schedule event
                        app_sched_event_put((void*)p_event, sizeof(ll_nvm_evt_t), lidar_config_nvm_general_handler);
                    }
                }
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        case FDS_EVT_UPDATE:
            // always clear busy flag
            m_nvm_busy = false;
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("<%d> NVM has been updated..",__LINE__);
                if (m_slave_address_update_pending == true)
                {
                    // install the secondary slave address for I2C
                    serial_twis_set_secondary_i2c_address(m_lidar_fpga_cfg.secondary_slave_address);
                    m_slave_address_update_pending = false;
                }
                // check for pending factory reset
                if ( m_pending_factory_reset == true)
                {
                    m_pending_factory_reset = false;
                    sd_nvic_SystemReset();
                }
            }
            else
            {
                // NOTE: no other details are available here, do nothing.
                // Errors from inside the lidar_config_nvm_update() function are returned to the
                // NVM event handler in main.c, the error code is NVM_UPDATE_FAILURE
            }

            break;

        default:
            break;
    }
}

static void lidar_config_nvm_general_handler(void *p_event_data, uint16_t size)
{
    ret_code_t err_code;
    ll_nvm_evt_t * p_event = (ll_nvm_evt_t *)p_event_data;

    switch(p_event->type)
    {
        case NVM_GARBAGE_COLLECTION:
            // start garbage collection routine
            err_code = fds_gc();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            NRF_LOG_WARNING("<%d> Unknown state hit in NVM general handler: %d",__LINE__, p_event->type);
            break;
    }

}

// Retrieve the secondary slave address from NVM
ret_code_t lidar_nvm_config_get_slave_address(uint8_t *slave_address)
{
    ret_code_t err_code = NRF_SUCCESS;
    if (m_fds_copy_in_memory == true)
    {
        *slave_address = m_lidar_fpga_cfg.secondary_slave_address;
    }
    else
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                *slave_address = m_lidar_fpga_cfg.secondary_slave_address;
                NRF_LOG_DEBUG("Getting slave address from NVM: 0x%02X",*slave_address);
            }
            else
            {
                NRF_LOG_ERROR("Error: lidar_config_get_slave_address() returned %s.\r\n", fds_err_str[err_code]);
                *slave_address = 0x00;  // disable on error
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    return err_code;
}

// write a new value to the NVM record
ret_code_t lidar_nvm_config_set_slave_address(uint8_t slave_address)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            // reset the boolean flag
            m_slave_address_update_pending = false;

            NRF_LOG_DEBUG("Writing slave address to NVM: %d",slave_address);
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new address
                m_lidar_fpga_cfg.secondary_slave_address = slave_address;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code == FDS_SUCCESS)
                {
                    m_slave_address_update_pending = true;
                }
                else
                {
                    // Because flash failed can not restart twi with requested address
                    m_slave_address_update_pending = false;
                    // address is not valid in memory, reset to zero
                    m_lidar_fpga_cfg.secondary_slave_address = 0x00;
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.secondary_slave_address = slave_address;
    }

    return err_code;
}

ret_code_t lidar_nvm_config_set_twis_mode(ant_i2c_modes_template_t mode)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new value for TWI slave mode
                m_lidar_fpga_cfg.i2c_enabled = mode;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.i2c_enabled = mode;
    }

    return err_code;
}

// return the TWIS mode
uint8_t lidar_nvm_config_get_twis_mode()
{
    uint8_t mode;
    if (m_fds_copy_in_memory == true)
    {
        mode = m_lidar_fpga_cfg.i2c_enabled;
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }

    return mode;
}

// return the power mode setting
uint8_t lidar_nvm_get_power_mode()
{
    // default value in case of failure
    lidar_lite_power_config_t mode = LIDAR_LITE_ALWAYS_ON;

    if (m_fds_copy_in_memory == true)
    {
        mode = m_lidar_fpga_cfg.power_mode;
        // write to register cache
        lidar_lite_set_library_register_value(LL_POWER_MODE, mode);
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }
    return mode;
}

// set the power mode setting
ret_code_t lidar_nvm_set_power_mode(lidar_lite_power_config_t mode)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new value for power mode
                m_lidar_fpga_cfg.power_mode = mode;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.power_mode = mode;
    }

    return err_code;
}

// return the measurement interval
uint8_t lidar_nvm_get_measurement_interval()
{
    uint8_t mode = DEFAULT_MEASUREMENT_INTERVAL;    // Default value
    if (m_fds_copy_in_memory == true)
    {
        mode = m_lidar_fpga_cfg.measurement_interval;
        // write to register cache
        lidar_lite_set_library_register_value(LL_MEASUREMENT_INTERVAL, mode);
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }
    return mode;
}

// set the measurement interval
ret_code_t lidar_nvm_set_measurement_interval(uint8_t mode)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new value for measurement_interval
                m_lidar_fpga_cfg.measurement_interval = mode;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.measurement_interval = mode;
    }

    return err_code;
}

ret_code_t lidar_nvm_set_power_and_interval(lidar_lite_power_config_t power_mode, uint8_t interval)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new value for power level and measurement_interval
                m_lidar_fpga_cfg.power_mode = power_mode;
                m_lidar_fpga_cfg.measurement_interval = interval;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.power_mode = power_mode;
        m_lidar_fpga_cfg.measurement_interval = interval;
    }

    return err_code;
}

// restore LLV4 to factory defaults.
// The default values are found in the header file and are applied
// on the initialization routine
ret_code_t lidar_nvm_factory_reset()
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new values for all NINE variables
                m_lidar_fpga_cfg.enable_nvm_storage         = DEFAULT_DISABLE_NVM_STORAGE;
                m_lidar_fpga_cfg.secondary_slave_address    = DEFAULT_SECONDARY_I2C_ADDR;
                m_lidar_fpga_cfg.i2c_enabled                = DEFAULT_I2C_CONFIGURATION;
                m_lidar_fpga_cfg.power_mode                 = DEFAULT_POWER_MODE;
                m_lidar_fpga_cfg.measurement_interval       = DEFAULT_MEASUREMENT_INTERVAL;
                m_lidar_fpga_cfg.detection_sensitivity      = DEFAULT_DETECTION_SENSITIVITY;
                m_lidar_fpga_cfg.acquisition_count          = DEFAULT_ACQUISITION_COUNT;
                m_lidar_fpga_cfg.quick_termination          = DEFAULT_QUICK_TERMINATION;
                m_lidar_fpga_cfg.high_accuracy_count        = DEFAULT_HIGH_ACCURACY_COUNT;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
                else
                {
                    m_pending_factory_reset = true;
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.enable_nvm_storage         = DEFAULT_DISABLE_NVM_STORAGE;
        m_lidar_fpga_cfg.secondary_slave_address    = DEFAULT_SECONDARY_I2C_ADDR;
        m_lidar_fpga_cfg.i2c_enabled                = DEFAULT_I2C_CONFIGURATION;
        m_lidar_fpga_cfg.power_mode                 = DEFAULT_POWER_MODE;
        m_lidar_fpga_cfg.measurement_interval       = DEFAULT_MEASUREMENT_INTERVAL;
        m_lidar_fpga_cfg.detection_sensitivity      = DEFAULT_DETECTION_SENSITIVITY;
        m_lidar_fpga_cfg.acquisition_count          = DEFAULT_ACQUISITION_COUNT;
        m_lidar_fpga_cfg.quick_termination          = DEFAULT_QUICK_TERMINATION;
        m_lidar_fpga_cfg.high_accuracy_count        = DEFAULT_HIGH_ACCURACY_COUNT;
    }

    return err_code;
}

// read the NVM value and return to caller
uint8_t lidar_nvm_get_detection_sensitivity()
{
    uint8_t level = 0x00;    // Default value
    if (m_fds_copy_in_memory == true)
    {
        level = m_lidar_fpga_cfg.detection_sensitivity;
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }
    return level;
}

// set the detection sensitivity level, from ANT module or I2C interface
ret_code_t lidar_nvm_set_detection_sensitivity(uint8_t sensitivity_level)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // set new value for power level and measurement_interval
                m_lidar_fpga_cfg.detection_sensitivity = sensitivity_level;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.detection_sensitivity = sensitivity_level;
    }

    return err_code;
}

uint8_t lidar_nvm_get_acquisition_count()
{
    uint8_t level = DEFAULT_ACQUISITION_COUNT;    // Default value
    if (m_fds_copy_in_memory == true)
    {
        level = m_lidar_fpga_cfg.acquisition_count;
        // write to register cache
        lidar_lite_set_library_register_value(LL_REGISTER_ACQUISITION_COUNT, level);
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }
    return level;
}

ret_code_t lidar_nvm_set_acquisition_count(uint8_t count)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                // store new value for acquisition count
                m_lidar_fpga_cfg.acquisition_count = count;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.acquisition_count = count;
    }


    return err_code;
}

ret_code_t lidar_nvm_set_quick_termination_mode(lidar_lite_termination_config_t mode)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
            if (err_code == FDS_SUCCESS)
            {
                m_lidar_fpga_cfg.quick_termination = mode;
                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else
            {
                NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // no NVM access, update RAM and return
        m_lidar_fpga_cfg.quick_termination = mode;
    }

    return err_code;
}

uint8_t lidar_nvm_get_quick_termination_mode()
{
    uint8_t mode = LIDAR_QUICK_TERMINATION_DISABLED;    // Default value
    if (m_fds_copy_in_memory == true)
    {
        mode = m_lidar_fpga_cfg.quick_termination;
        // write to register cache
        lidar_lite_set_library_register_value(LL_QUICK_TERMINATION, mode );
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }

    return mode;
}

ret_code_t lidar_nvm_set_high_accuracy_count(uint8_t count)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (lidar_nvm_get_access_mode() == true)
    {
        if (m_nvm_busy == false)
        {
            if (m_lidar_fpga_cfg.power_mode == LIDAR_LITE_ALWAYS_ON)
            {
                err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
                if (err_code == FDS_SUCCESS)
                {
                    m_lidar_fpga_cfg.high_accuracy_count = count;
                    m_nvm_busy = true;
                    err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                    if (err_code != FDS_SUCCESS)
                    {
                        NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                    }
                }
                else
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
            else    // can't set because it is not in correct power mode
            {
                err_code = NRF_ERROR_INVALID_STATE;
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else
    {
        // NO NVM access, update RAM and return
        m_lidar_fpga_cfg.high_accuracy_count = count;
    }

    return err_code;
}

uint8_t lidar_nvm_get_high_accuracy_count()
{
    uint8_t count = DEFAULT_HIGH_ACCURACY_COUNT;    // Default value
    if (m_fds_copy_in_memory == true)
    {
        count = m_lidar_fpga_cfg.high_accuracy_count;
        // write to register cache
        lidar_lite_set_library_register_value(LL_HIGH_ACCURACY_MODE, count);
    }
    else
    {
        NRF_LOG_ERROR("FDS should already be in memory");
    }

    return count;
}

bool lidar_nvm_get_access_mode()
{
    if (m_lidar_fpga_cfg.enable_nvm_storage == NVM_STORAGE_ENABLED)
    {
        // write to register cache
        lidar_lite_set_library_register_value(LL_NVM_ACCESS_MODE, NVM_STORAGE_ENABLED);
        return true;
    }
    else
    {
        // write to register cache
        lidar_lite_set_library_register_value(LL_NVM_ACCESS_MODE, NVM_STORAGE_DISABLED);
        return false;
    }
}

ret_code_t lidar_nvm_set_access_mode(uint8_t mode)
{
    ret_code_t err_code = FDS_SUCCESS;

    if (mode == NVM_STORAGE_ENABLED)
    {
        if (m_nvm_busy == false)
        {
            // only write if NVM access is currently disabled
            if (lidar_nvm_get_access_mode() == false)
            {
                err_code = lidar_config_nvm_read(&m_lidar_fpga_cfg);
                if (err_code == FDS_SUCCESS)
                {
                    m_lidar_fpga_cfg.enable_nvm_storage = mode;
                    m_nvm_busy = true;
                    err_code = lidar_config_nvm_update(&m_lidar_fpga_cfg);
                    if (err_code != FDS_SUCCESS)
                    {
                        NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_update() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                    }
                }
                else
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_read() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }
    else    // DISABLE mode has been triggered
    {
        if (m_nvm_busy == false)
        {
            if (lidar_nvm_get_access_mode() == true)
            {
                // create a copy of the existing configuration in RAM
                // write this to the NVM
                // disable the access
                m_lidar_fpga_cfg.enable_nvm_storage = DEFAULT_DISABLE_NVM_STORAGE;
                memcpy(&m_temp_lidar_fpga_cfg, &m_lidar_fpga_cfg, sizeof(m_lidar_fpga_cfg));

                m_nvm_busy = true;
                err_code = lidar_config_nvm_update(&m_temp_lidar_fpga_cfg);
                if (err_code != FDS_SUCCESS)
                {
                    NRF_LOG_ERROR("<%d> Error: lidar_config_nvm_udpate() returned %s.\r\n", __LINE__, fds_err_str[err_code]);
                }
            }
        }
        else
        {
            err_code = FDS_ERR_BUSY;
        }
    }

    return err_code;
}
