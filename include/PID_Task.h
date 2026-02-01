#ifndef PID_TASK_H
#define PID_TASK_H

#include <Arduino.h> // Acá está el FreeRTOS, ya tiene su ifndefs internos

// Prototipo de la función de la tarea PID
void TaskPIDCode(void * pvParameters);

#endif