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

#ifndef __SOC_LOG_DEFINES__
#define __SOC_LOG_DEFINES__

// Log levels used by SOC. These should be redefined to whatever constants
// should be used by the logging backend. By default they are integers with
// lower numbers indicating higher priority.
#define SOC_LOG_LEVEL_ERROR     1
#define SOC_LOG_LEVEL_WARN      2
#define SOC_LOG_LEVEL_INFO      3
#define SOC_LOG_LEVEL_DEBUG     4

// call back routine for SOC log handler.
typedef void (*soc_log_handler_t) (uint32_t, const char* string, ...);

#endif // __SOC_LOG_DEFINES__