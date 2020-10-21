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

#include "app_error.h"
#include "app_scheduler.h"
#include "hardfault.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_soc.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ant_ranging_profile.h"
#include "ant_message_parser.h"
#include "app_interface.h"
#include "lidar_config_nvm.h"
#include "lidar_lite_interface.h"
#include "serial_twis.h"
#include "serial_twis_parser.h"
#include "ant_interface.h"
#include "soc_logger.h"

#define NRF_LOG_MODULE_NAME main
#if NRF_LOG_ENABLED && MAIN_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       MAIN_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL       0
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();


#define APP_ANT_OBSERVER_PRIO 1 /**< Application's ANT observer priority. You shouldn't need to modify this value. */

// Scheduler settings
// Update the data size as other elements are added to the scheduler (must be the largest one!)
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(MAX(MAX(NRF_SDH_ANT_EVT_BUF_SIZE, sizeof(nrf_drv_twis_evt_t)), MAX(sizeof(nrfx_spim_evt_t), sizeof(nrf_drv_gpiote_pin_t))), sizeof(fds_evt_t))
#define SCHED_QUEUE_SIZE                50

#define APP_INPUT_A_PIN                 NRF_GPIO_PIN_MAP(0, 4)
#define APP_INPUT_A_PIN_SENSE           NRF_GPIOTE_POLARITY_TOGGLE
#define APP_INPUT_A_PIN_PULL            NRF_GPIO_PIN_PULLDOWN

#define APP_INPUT_B_PIN                 NRF_GPIO_PIN_MAP(0, 5)
#define APP_INPUT_B_PIN_SENSE           NRF_GPIOTE_POLARITY_TOGGLE
#define APP_INPUT_B_PIN_PULL            NRF_GPIO_PIN_NOPULL

// High speed distance mode PINS
#define APP_OUTPUT_B_PIN_LIB_BUSY               NRF_GPIO_PIN_MAP(0, 5)
#define APP_INPUT_A_PIN_TRIGGER_MEASUREMENT     NRF_GPIO_PIN_MAP(0, 4)

// Local Functions
static void app_input_pin_event_handler(void * p_event_data, uint16_t event_size);
static void app_apply_NVM_configurations();
static void configure_secondary_i2c_address();
static void check_power_mode_settings();

// synchronization variables and states for ensuring NVM_INIT_COMPLETE and LIDAR_LITE_EVT_POWER_UP_COMPLETE
// are done before applying the configurations from the NVM / flash file system
static uint8_t m_synchronize_callbacks = 0;
#define NVM_READY            0x10
#define POWER_UP_FINISHED    0x20
#define SYNCH_COMPLETE       NVM_READY + POWER_UP_FINISHED
// end of synchronization variables and states

static lidar_measurement_state_t m_measurement_state = NO_MEASUREMENT_IN_PROGRESS;

/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
static void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    ant_rp_evt_handler(p_ant_evt, p_context);
}

/**@brief Function for handling Lidar Lite NVM events
 *
 * @param[in] p_event  Lidar Lite NVM event.
 */
static void lidar_lite_nvm_handler(ll_nvm_evt_t * p_event)
{
    ll_nvm_evt_type_t event_type = p_event->type;
    uint8_t ret_code = p_event->response_code;

    switch(event_type)
    {
        case NVM_INIT_COMPLETE:
            NRF_LOG_INFO("NVM has been intialized and Configurations are stored in memory");
            m_synchronize_callbacks |= NVM_READY;
            app_apply_NVM_configurations();
            break;

        case NVM_UPDATE_FAILURE:     // Updates to FDS have failed.
            NRF_LOG_ERROR("NVM update failure has been detected in FDS storage module");
            break;

        default:
           NRF_LOG_WARNING("Received unexpected NVM Event Type: %d Response Code: %d", event_type, ret_code);
            break;
    }
}

/**@brief Function for handling Lidar Lite Library events.
 *
 * @param[in] p_event  Lidar Lite Library Event.
 */
static void lidar_lite_evt_handler(lidar_lite_evt_t * p_event)
{

    lidar_lite_evt_type_t event_type = p_event->type;
    lidar_lite_evt_response_code_t response_code = p_event->response_code;

    switch(event_type)
    {
        case LIDAR_LITE_EVT_POWER_UP_COMPLETE:
            if(response_code != LIDAR_LITE_COMMAND_SUCCESSFUL)
            {
                NRF_LOG_ERROR("FPGA Power Complete Event failed with response_code %d.", response_code);
                return;
            }
            NRF_LOG_INFO("FPGA Power up cycle complete, the FPGA is ready to accept commands.");
            m_synchronize_callbacks |= POWER_UP_FINISHED;
            app_apply_NVM_configurations();

            break;
        case LIDAR_LITE_EVT_READ_COMPLETE:
            if(response_code != LIDAR_LITE_COMMAND_SUCCESSFUL)
            {
                NRF_LOG_ERROR("Read Complete Event failed with response_code %d.", response_code);
                return;
            }
            uint8_t fpga_register_address = p_event->data.fpga_register.fpga_register_address;
            uint8_t fpga_register_value = p_event->data.fpga_register.fpga_register_value;
            NRF_LOG_DEBUG("Lidar Lite Event Read Complete. Register 0x%02x has value 0x%02x", fpga_register_address, fpga_register_value);
            break;

        case LIDAR_LITE_EVT_MEASUREMENT_COMPLETE:
            NRF_LOG_DEBUG("Distance measurement complete.");
            uint16_t distance_cm;
            if(response_code != LIDAR_LITE_COMMAND_SUCCESSFUL)
            {
                distance_cm = ANT_RP_MEASUREMENT_FAILED;
                NRF_LOG_WARNING("Measurement failed with response code %d", response_code);
            }
            else
            {
                distance_cm = p_event->data.distance_cm;
                NRF_LOG_INFO("Measured Distance: %d", distance_cm);
            }

            measurement_completed();

            // Update the distance transmitted over ANT
            ant_rp_update_distance(distance_cm);
            break;

        case LIDAR_LITE_EVT_TEMP_COMPLETE:
        {
            int8_t board_temp;
            if (response_code == LIDAR_LITE_COMMAND_SUCCESSFUL)
            {
                board_temp = p_event->data.board_temperature;
                NRF_LOG_INFO("Temperature measurement complete: %d", board_temp);
            }
            break;
        }

        default:
            NRF_LOG_WARNING("Received unexpected Lidar Lite Event Type: %d Response Code: %d", event_type, response_code);
            break;

    }
}


/**@brief   Function for scheduling interrupt line events from the application defined input pin.
 */
static void app_input_pin_event_schedule(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_pin_t * p_event = &pin;
    app_sched_event_put((void*)p_event, sizeof(nrf_drv_gpiote_pin_t), app_input_pin_event_handler);
}

/**@brief   Function for handling interrupt line events from the application defined input pin.
 */
static void app_input_pin_event_handler(void * p_event_data, uint16_t event_size)
{
    nrf_drv_gpiote_pin_t * p_event = (nrf_drv_gpiote_pin_t*) p_event_data;
    nrf_drv_gpiote_pin_t pin = p_event[0];

    bool pin_state = nrf_drv_gpiote_in_is_set(pin);

    NRF_LOG_INFO("Input Pin Interrupt Event. Triggered by pin: %d with current state: %d", pin, pin_state);

    // GPIO signal detected handle event
    if (pin == APP_INPUT_A_PIN_TRIGGER_MEASUREMENT)
    {
        if (lidar_lite_request_measurement(true) == LIDAR_LITE_SUCCESS)    // start distance measurement
        {
            measurement_started();
        }
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

/**@brief Function for ANT stack initialization.
 */
static void softdevice_setup(void)
{
    uint32_t clock_is_running;

    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    // enable high speed clock for TRACE0 pin.
    err_code = sd_clock_hfclk_request();
    APP_ERROR_CHECK(err_code);
    do
    {
        sd_clock_hfclk_is_running(&clock_is_running);
    }while(clock_is_running == 0);

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the application's GPIOs
 * For development purposes, using GPIOTE to detect changes in the state
 * of a GPIO. The current state of the GPIO is transmitted in the payload of the ANT
 * master channel to validate board functionality.  It would be possible to simply
 * sample the state of the pin before transmitting, however, we are using GPIOTE for
 * now to ensure that the application can use GPIOTE even with the library using it too.
 */
static void application_gpio_setup(void)
{
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
      err_code = nrf_drv_gpiote_init();
      APP_ERROR_CHECK(err_code);
    }

    // Configure the GPIO pins
    nrf_drv_gpiote_in_config_t config_input_a =
    {
        .sense = APP_INPUT_A_PIN_SENSE,
        .pull = APP_INPUT_A_PIN_PULL,
        .is_watcher = false,
        .hi_accuracy = false
    };

    // Initialize the GPIO pins
    err_code = nrf_drv_gpiote_in_init(APP_INPUT_A_PIN_TRIGGER_MEASUREMENT, &config_input_a, app_input_pin_event_schedule);
    APP_ERROR_CHECK(err_code);

    // Enable the GPIO pins
    nrf_drv_gpiote_in_event_enable(APP_INPUT_A_PIN_TRIGGER_MEASUREMENT, false);

    // configure once as output
    nrf_gpio_cfg_output(APP_OUTPUT_B_PIN_LIB_BUSY);
}


/**@brief Function to read and install secondary I2C slave address
 */
static void configure_secondary_i2c_address()
{
    ret_code_t err_code;
    uint8_t slave_address, i2c_mode;

    // must check the state of the I2C enable/disable flag
    // if disabled stop I2C
    i2c_mode = lidar_nvm_config_get_twis_mode();
    if (i2c_mode == I2C_DISABLED)
    {
        NRF_LOG_INFO("I2C has been disabled over ANT");
        serial_twis_disable();
        // put the softdevice into low power mode.
        sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    }
    else
    {
        err_code = lidar_nvm_config_get_slave_address(&slave_address);
        if (err_code == FDS_SUCCESS)
        {
            err_code = serial_twis_set_secondary_i2c_address(slave_address);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Secondary I2C address configured: 0x%02X", slave_address);

                // If secondary I2C address is configured and the address is valid, set the I2C_CONFIG
                // register to LL_BOTH_ADDRESSES
                if( slave_address != LL_I2C_DEACTIVATED )
                {
                    err_code = serial_twis_configure_i2c_addresses(LL_BOTH_ADDRESSES);
                    if(err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_DEBUG("Configure I2C_CONFIG to LL_BOTH_ADDRESSES FAILED");
                    }
                }
            }
            else
            {
                NRF_LOG_ERROR("Secondary I2C address could not be set");
            }
        }
        else
        {
            NRF_LOG_ERROR("Slave address retrieval from NVM was not successful.");
        }
    }
}

// read the NVM power level and send to library and ANT
static void check_power_mode_settings()
{
    ret_code_t err_code;
    uint8_t power_mode;
    lidar_lite_power_config_t system_power_mode = LIDAR_LITE_ALWAYS_ON;  // default power mode on startup

    // fetch the power mode from the NVM
    // only need to adjust if NOT LIDAR_LITE_ALWAY_ON
    power_mode = lidar_nvm_get_power_mode();
    if (power_mode != system_power_mode)
    {
        // halt any SYNCHRONOUS mode operations
        ant_rp_power_mode_reset();

        // pass information to Library, always returns SUCCESS
        err_code = lidar_lite_configure_power_mode(power_mode);
        if (err_code == LIDAR_LITE_SUCCESS)
        {
            // inform ANT
            ant_rp_power_mode_update(power_mode);
        }
        else
        {
            NRF_LOG_INFO("Power mode could not be applied to system: %d",err_code);
        }
    }
}

// read the NVM measurement interval delay and send to ANT
static void check_measurement_interval_settings()
{
    uint8_t stored_interval;

    stored_interval = lidar_nvm_get_measurement_interval();
    ant_rp_measurement_interval_udpate(stored_interval);
}

static void app_apply_NVM_configurations()
{
    // only apply NVM configurations if both callback events are signaled
    // Power up complete is done and the NVM contents are in memory
    if (m_synchronize_callbacks == SYNCH_COMPLETE)
    {
        uint8_t current_setting;
        ret_code_t err_code;

        // Check and install secondary I2C address
        configure_secondary_i2c_address();

        check_power_mode_settings();

        check_measurement_interval_settings();

        // pass detection sensitivity to Library
        current_setting = lidar_nvm_get_detection_sensitivity();
        err_code = lidar_lite_set_detection_sensitivity(current_setting);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Sensitivity level could not be set in Library");
        }

        // pass acquisition count to library
        current_setting = lidar_nvm_get_acquisition_count();
        err_code = lidar_lite_set_acquisition_count(current_setting);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Acquistion count could not be set in Library");
        }

        // pass Quick termination mode to Library
        current_setting = lidar_nvm_get_quick_termination_mode();
        err_code = lidar_lite_set_quick_termination_mode(current_setting);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Quick termination mode could not be set in Library");
        }

        // High accuracy count / Mode
        current_setting = lidar_nvm_get_high_accuracy_count();
        err_code = lidar_lite_set_high_accuracy_count(current_setting);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("High accuracy mode could not be set in Library");
        }

        // reset to Zero as we only want to snych once after power up
        m_synchronize_callbacks = 0;
    }
}

uint32_t get_esn(void)
{
    uint32_t esn = NRF_UICR->CUSTOMER[SYSTEM_UICR_CUST_ESN_OFFSET];
    return esn;
}

uint32_t get_ant_id(void)
{
    uint32_t ant_id = NRF_UICR->CUSTOMER[SYSTEM_UICR_CUST_ANT_ID_OFFSET];
    return ant_id;
}

// The Lidar Lite Library error handler.
void library_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    volatile bool loop = true;

    NRF_LOG_INFO("Library Error: Code: %d, Line: %d, File: %s",error_code, line_num, p_file_name);

    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    while(loop);
#endif // DEBUG
}

// Handle all LOG "strings" from the Lidar Lite Library.
void log_handler(uint32_t LEVEL, const char* string, ...)
{
#if defined(DEBUG)
    va_list argptr;
    va_start(argptr, string);
    switch (LEVEL)
    {
        case SOC_LOG_LEVEL_ERROR:
            NRF_LOG_ERROR("%s", string);
            break;

        case SOC_LOG_LEVEL_WARN:
            NRF_LOG_WARNING("%s", string);
            break;

        case SOC_LOG_LEVEL_INFO:
            NRF_LOG_INFO("%s", string);
            break;

        case SOC_LOG_LEVEL_DEBUG:
            NRF_LOG_DEBUG("%s", string);
            break;

        default:  NRF_LOG_ERROR("<%d> Log Level not recognized", __LINE__);
            break;
    }
    NRF_LOG_FLUSH();
#endif
}

void measurement_started(void)
{
    m_measurement_state = MEASUREMENT_IN_PROGRESS;
    nrf_gpio_pin_set(APP_OUTPUT_B_PIN_LIB_BUSY);    // Drive pin High, logic level 1
}


void measurement_completed(void)
{
    nrf_gpio_pin_clear(APP_OUTPUT_B_PIN_LIB_BUSY);    // Drive pin low, logic level zero
    m_measurement_state = NO_MEASUREMENT_IN_PROGRESS;
}

lidar_measurement_state_t get_measurement_state(void)
{
    return m_measurement_state;
}

/**@brief Function for application main entry. Does not return.
 */
int main(void)
{

    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    softdevice_setup();
    ant_rp_init();

    // Initializing Lidar Config NVM install callback handler for returning
    // Messages back to main
    err_code = lidar_config_nvm_init(lidar_lite_nvm_handler);
    APP_ERROR_CHECK(err_code);

    // Initialize the LIDAR Lite Library; use scheduler, use custom library error handler and log handler defined above.
    err_code = lidar_lite_init(true, lidar_lite_evt_handler, library_error_handler, log_handler);
    APP_ERROR_CHECK(err_code);

    // Initialize I2C interface with external MCU; use the scheduler to handle I2C events
    err_code = lidar_lite_serial_twis_init(lidar_lite_serial_twis_event_schedule);
    APP_ERROR_CHECK(err_code);

    // Initialize the applications GPIO
    application_gpio_setup();

    // set the power mode so that the PPI I2C will work as expected.
    sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);

    NRF_LOG_INFO("LIDAR-Lite Initialized");

    // Main loop.
    for (;;)
    {
        app_sched_execute();
        NRF_LOG_FLUSH();

        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}
