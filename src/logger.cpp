#include "logger.h"
#include "global.h"
#include "config.h"


static inline double iden_time_sec(void) {
    return (double)iden_counter * TS_SEC; // tiempo real en segundos
}


void handleLogger() {
    
    if (!b_logging) return; // Si el logging no está activo, salgo
    
    // millis() está definido en esp32-hal.h y es la forma correcta de obtener el tiempo en ms con freeRTOS  
    // now: timestamp auxiliar (debug / CSV).
    // El tiempo real del sistema es k * TS_SEC.
    //unsigned long now = millis();
    

    double duty_visual = (double)duty_applied / 65535.0 * 100.0; // Convertir a porcentaje para logging
    if (digitalRead(SENTIDO_HORARIO) == HIGH) {
        duty_visual = -duty_visual;
    }

    if(g_iden_mode != IDEN_NONE){
        // En modo identificación, solo guardo tiempo, posición y duty
        Serial.printf("%.3f,%.4f,%.2f\n", 
                        iden_time_sec(), 
                        posicion_actual, 
                        duty_visual);
        return;
    }else{
        // En modo normal de control PID, guardo todo
        // intento obtener formato CSV: Tiempo, Setpoint, Posición, Error, Duty(PWM)
            
        double error_visual = setpoint - posicion_actual; //Variable auxiliar para el error
            
        Serial.printf("%.3f,%.3f,%.3f,%.3f,%.2f\n", 
                        iden_time_sec(), 
                        setpoint, 
                        posicion_actual, 
                        error_visual, 
                        duty_visual);

    }        
    
}

void TaskLoggerCode(void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {

        handleLogger(); //La tarea se encarga de loggear si la bandera está activa
        vTaskDelayUntil(&lastWakeTime, TS_TICKS);

    }
}
