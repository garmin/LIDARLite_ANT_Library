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

#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "soc_error_handler.h"

#ifdef USE_SOC_LIBRARY_APP_ERROR
soc_lib_error_handler_t m_error_handler = NULL;

void lib_error_handler_set(soc_lib_error_handler_t lib_error_handler)
{
    m_error_handler = lib_error_handler;
}

void lib_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    uint8_t const * p_file_name_stripped;

    // Strip out the path and only return the file name
    p_file_name_stripped = (uint8_t*)(strrchr((char *)p_file_name, '/')); // Search for last occurrence of '/'

    if(p_file_name_stripped != NULL)
    {
        p_file_name_stripped += 1; // Point to the next character after the slash
    }
    else
    {
        p_file_name_stripped = p_file_name; // If no slash found, use full file name
    }

    // Call application error handler
    if(m_error_handler != NULL)
    {
        m_error_handler(error_code, line_num, p_file_name_stripped);
    }
}
#endif