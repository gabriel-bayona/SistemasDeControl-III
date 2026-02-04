#ifndef LOGGER_H
#define LOGGER_H
#include <Arduino.h>

// Función principal para gestionar el envío de datos
void handleLogger();

//Funcion auxiliar para obtener el tiempo en segundos de identificación (multiplo de TS_SEC)
static inline double iden_time_sec(void);
//Usar static inline asegura que la función sea visible solo en la unidad de traducción actual, evitando conflictos de enlazado.

// Tarea FreeRTOS para logging
void TaskLoggerCode(void *pvParameters);
#endif