/*
Copyright (c) 2019 Garmin Ltd. or its subsidiaries.

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
#include "nrf_drv_timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lidar_lite_interface.h"
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

#define SCHED_MAX_EVENT_DATA_SIZE       32
#define SCHED_QUEUE_SIZE                50

// timer constants
const nrf_drv_timer_t TIMER_MEASURE = NRF_DRV_TIMER_INSTANCE(0);

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
            break;

        case LIDAR_LITE_EVT_MEASUREMENT_COMPLETE:
            {
                uint16_t distance_cm;
                if(response_code != LIDAR_LITE_COMMAND_SUCCESSFUL)
                {
                    NRF_LOG_WARNING("Measurement failed with response code %d", response_code);
                }
                else
                {
                    distance_cm = p_event->data.distance_cm;
                    NRF_LOG_INFO("Measured Distance: %d", distance_cm);
                }
            }
            break;

        default:
            NRF_LOG_WARNING("Received unexpected Lidar Lite Event Type: %d Response Code: %d", event_type, response_code);
            break;
    }
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

void timer_measurement_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    lidar_lite_return_t ret_code;

    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            ret_code = lidar_lite_request_measurement(true);
            APP_ERROR_CHECK(ret_code);
            break;
        default:
            // do nothing
            break;
    }
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

/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t time_ms = 2500;    // 2.5 seconds
    uint32_t time_ticks;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Initialize the LIDAR Lite Library; use scheduler, use custom library error handler defined above.
    err_code = lidar_lite_init(true, lidar_lite_evt_handler, library_error_handler, log_handler);
    APP_ERROR_CHECK(err_code);

    // setup timer
    err_code = nrf_drv_timer_init(&TIMER_MEASURE, &timer_cfg, timer_measurement_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_MEASURE, time_ms);

    nrf_drv_timer_extended_compare(&TIMER_MEASURE, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_MEASURE);

    NRF_LOG_INFO("Example Application Initialized");

    // Main loop, no return
    for (;;)
    {
        app_sched_execute();
        NRF_LOG_FLUSH();
    }
}
