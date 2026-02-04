#include "global.h"
#include "config.h"
#include "PID_Task.h"
#include "menu.h"
#include "logger.h"

//este setup solo afecta al nucleo 1, me conviene usarlo para la comunicación serial
void setup() {

  Serial.begin(115200);
  
  initPeripherals(); // Inicializa hardware (pines, PWM, encoder, etc, etc, etc)
  Serial.println("Sistema Dual Core...");

  // CREAR TAREA EN NÚCLEO 0
    xTaskCreatePinnedToCore(
                    TaskPIDCode,   // Función de la tarea
                    "PID_Task",    // Nombre de la tarea 
                    10000,         // Tamaño de pila
                    NULL,          // Parámetros
                    1,             // Prioridad (1 es suficiente, >0)
                    &TaskPID_Handle, // Handle
                    0);            // NÚCLEO 0 (El loop corre en el 1)    
                    
// CREAR TAREA DE LOGGER EN NÚCLEO 1
    xTaskCreatePinnedToCore(
                    TaskLoggerCode,  // Función de la tarea
                    "Logger_Task",   // Nombre de la tarea
                    5000,            // Tamaño de pila
                    NULL,             // Parámetros
                    1,              // Prioridad
                    NULL,           // Handle (no necesito)
                    1);  // núcleo 1 (junto al loop / serial)
    

}


//LOOP (NÚCLEO 1 - lo uso solo para comunicación)
void loop() {
  
    handleSerialMenu();
    // Un pequeño delay para no saturar el Núcleo 1 innecesariamente
    // El loop puede ir lento, no afecta al PID, no necesito alta frecuencia aquí, por eso uso delay
    delay(50);
}