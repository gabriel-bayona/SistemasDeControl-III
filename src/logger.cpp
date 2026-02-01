#include "logger.h"
#include "global.h"
#include "config.h"

void handleLogger() {
    // Salgo si el logging está desactivado
    if (!b_logging) return;

    static unsigned long lastLogTime = 0;
    unsigned long now = millis(); // millis() está definido en esp32-hal.h y es la forma correcta de obtener el tiempo en ms con freeRTOS
    const int LOG_INTERVAL = 10; // Guardar datos cada 10ms

    if (now - lastLogTime >= LOG_INTERVAL) {
        lastLogTime = now;
        
        double duty_visual = (double)duty_global / 65535.0 * 100.0; // Convertir a porcentaje para logging
        if (digitalRead(SENTIDO_HORARIO) == HIGH) {
            duty_visual = -duty_visual;
        }

        if(b_identification || b_identification2){
            // En modo identificación, solo guardo tiempo, posición y duty
            Serial.printf("%lu,%.4f,%.2f\n", 
                          now, 
                          posicion_actual, 
                          duty_visual);
            return;
        }else{
            // En modo normal de control PID, guardo todo
            // intento obtener formato CSV: Tiempo, Setpoint, Posición, Error, Duty(PWM)
            
            double error_visual = setpoint - posicion_actual; //Variable auxiliar para el error
            
            Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f\n", 
                           now, 
                           setpoint, 
                           posicion_actual, 
                           error_visual, 
                           duty_visual);

        }        
    }
}