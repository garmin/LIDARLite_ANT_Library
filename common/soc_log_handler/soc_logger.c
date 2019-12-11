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

#include "soc_logger.h"
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

soc_log_handler_t m_log_event = NULL;
void soc_log_init (void (*ptr)(uint32_t, const char* string, ...))
{
    m_log_event = ptr;
}

void soc_log(
        char* name,
        uint32_t level,
        const char* msg,
        ...
        )
{
    char temp[MAX_SOC_LOG_BUFFER];

    // check for overflow
    if (strlen( name ) + strlen( msg ) < MAX_SOC_LOG_BUFFER)
    {
        sprintf(temp, name);
        sprintf(temp + strlen(name), msg);
        if(m_log_event != NULL)
        {
            (*m_log_event)(level, temp);
        }
    }
    else
    {
        // overflow error, generate custom message
        if(m_log_event != NULL)
        {
            sprintf(temp,"%s, Log message from module exceeds buffer limit: %d",name, MAX_SOC_LOG_BUFFER);
            (*m_log_event)(level, temp);
        }
    }
}

void soc_log_hexdump_module
        (
        char* name,
        uint32_t level,
        uint8_t p_data[],
        uint8_t len
        )
{
}