#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H
#include "FreeRTOS.h"
#include "task.h"


extern TaskHandle_t xModbusTaskHandle;


void Modbus_Server_Init(void);



#endif
