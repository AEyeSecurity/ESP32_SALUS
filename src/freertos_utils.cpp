#include "freertos_utils.h"

#include "ota_telnet.h"

bool startTaskPinned(TaskFunction_t task,
                     const char* name,
                     uint16_t stackSize,
                     void* parameter,
                     UBaseType_t priority,
                     TaskHandle_t* handle,
                     const BaseType_t coreId) {
  BaseType_t result = xTaskCreatePinnedToCore(task, name, stackSize, parameter, priority, handle, coreId);
  if (result != pdPASS) {
    String msg = String("No se pudo crear la tarea ") + name;
    EnviarMensajeTelnet(msg);
    return false;
  }
  return true;
}
