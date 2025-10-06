#ifndef FREERTOS_UTILS_H
#define FREERTOS_UTILS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

bool startTaskPinned(TaskFunction_t task,
                     const char* name,
                     uint16_t stackSize,
                     void* parameter,
                     UBaseType_t priority,
                     TaskHandle_t* handle,
                     const BaseType_t coreId);

#endif  // FREERTOS_UTILS_H
