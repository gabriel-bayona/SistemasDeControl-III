#include "identification.h"
#include "global.h"
#include "config.h"


void runIdentificationStep() {
    if (!b_identification) return;

    float duty = 0.0;
    float max_duty_percent = 90.0; // Máximo duty cycle en porcentaje
    
    // Definimos la duración de cada "pata" del triángulo en ticks del contador
    // Ejemplo: 0 a 300 sube, 300 a 600 baja.
    int t_subida = 300; 
    int t_bajada = 600;  // Punto donde termina la bajada (300 de subida + 300 de bajada)
    int t_cambio = 600;  // Punto donde cambiamos de sentido
    
    // PRIMERA ETAPA: SENTIDO ANTIHORARIO (0 a 600 ticks)
    if (iden_counter < t_cambio) {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);

        if (iden_counter < t_subida) {
            // Rampa de subida (0% a max%)
            // Ecuación lineal: y = (x / x_max) * y_max
            duty = ((float)iden_counter / (float)t_subida) * max_duty_percent;
        } else {
            // Rampa de bajada (max% a 0%)
            // Ecuación: mapeamos de t_subida->t_bajada a max->0
            float pasos_en_bajada = iden_counter - t_subida;
            float duracion_bajada = t_bajada - t_subida;
            duty = max_duty_percent * (1.0 - (pasos_en_bajada / duracion_bajada));
        }
        
        // Actualizamos duty global positivo para indicar Antihorario
        duty_global = (int32_t)duty; 

    // SEGUNDA ETAPA: SENTIDO HORARIO (600 a 1200 ticks)
    } else if (iden_counter >= t_cambio && iden_counter < (t_cambio * 2)) {
        digitalWrite(SENTIDO_HORARIO, HIGH);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);

        int local_counter = iden_counter - t_cambio; // Reiniciamos contador virtual para esta etapa

        if (local_counter < t_subida) {
            // Rampa subida
            duty = ((float)local_counter / (float)t_subida) * max_duty_percent;
        } else {
            // Rampa bajada
            float pasos_en_bajada = local_counter - t_subida;
            float duracion_bajada = t_bajada - t_subida;
            duty = max_duty_percent * (1.0 - (pasos_en_bajada / duracion_bajada));
        }

        // Actualizamos duty global
        duty_global = (int32_t)duty* 65535 / 100; // Escalamos a 16 bits

    // FINALIZACIÓN
    } else {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
        
        duty_global = 0;
        b_identification = false;
        b_logging = false; 
        iden_counter = 0; // Reseteamos para la próxima vez
        Serial.println("FIN IDENTIFICACION");
        return; 
    }
    // Saturación de duty
    if (duty > 100.0) duty = 100.0;

    // Mapeo de Porcentaje (0-100) a 16 bits (0-65535)
    // Usamos float para no perder precisión en la división
    uint32_t raw_pwm = (uint32_t)((duty / 100.0) * 65535.0); 

    // Saturación de seguridad para 16 bits
    if (raw_pwm > 65535) raw_pwm = 65535;

    // Escribir al PWM
    if(pwm) pwm->setPWM_Int(PWM_PIN, FRECUENCIA, raw_pwm);
    //if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    iden_counter++;
}


void runIdentificationStep2() {
    if (!b_identification2) return;

    float duty = 0.0;
    float max_duty_percent = 90.0; // Máximo duty cycle en porcentaje
    
    // Definimos la duración de cada escalón en ticks del contador
    int step_duration = 200; // Duración de cada escalón en ticks
    int total_steps = 6;     // Total de escalones (3 positivos, 3 negativos)
    
    int current_step = iden_counter / step_duration;
    
    if (current_step < total_steps) {
        if (current_step % 2 == 0) {
            // Escalón positivo (Sentido Antihorario)
            digitalWrite(SENTIDO_HORARIO, LOW);
            digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
            duty = max_duty_percent;
        } else {
            // Escalón negativo (Sentido Horario)
            digitalWrite(SENTIDO_HORARIO, HIGH);
            digitalWrite(SENTIDO_ANTIHORARIO, LOW);
            duty = max_duty_percent;
        }

        // Actualizamos duty global
        duty_global = (int32_t)duty * 65535 / 100; // Escalamos a 16 bits

    } else {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
        
        duty_global = 0;
        b_identification2 = false;
        b_logging = false; 
        iden_counter = 0; // Reseteamos para la próxima vez
        Serial.println("FIN IDENTIFICACION 2");
        return; 
    }

    // Mapeo de Porcentaje (0-100) a 16 bits (0-65535)
    uint32_t raw_pwm = (uint32_t)((duty / 100.0) * 65535.0); 

    // Saturación de seguridad para 16 bits
    if (raw_pwm > 65535) raw_pwm = 65535;

    // Escribir al PWM
    if(pwm) pwm->setPWM_Int(PWM_PIN, FRECUENCIA, raw_pwm);

    iden_counter++;
}

/*
void runIdentificationStep() {
    //Verifico si corresponde la ejecución
    if (!b_identification) return;

    double duty = 0;
    
    // Genero la señal de identificación en forma de rampa escalonada
    if(iden_counter < 300){
        // Rampa positiva (Sentido Antihorario)
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
        duty = iden_counter * 2.5; 

    } else if (iden_counter >= 300 && iden_counter <= 500) {
        // Rampa negativa (Sentido Horario)
        digitalWrite(SENTIDO_HORARIO, HIGH);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        duty = 70 - iden_counter; 

    } else if (iden_counter > 500 && iden_counter < 700) {
        // Rampa positiva de nuevo
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
        duty = iden_counter - 500;

    } else {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
        
        b_identification = false;
        b_logging = false; 
        Serial.println("FIN IDENTIFICACION");
        return; 
    }

    duty_global = (int32_t)duty;
    // Configuro el PWM correspondiente
    uint32_t raw_pwm = (uint32_t)((duty / 100.0) * 1023.0); //Recordar que el PWM es de 10 bits (0-1023) y tengo que mapear
                                                            //Si no recuerdo, ir a ver la librería ESP32_FastPWM

    
    // Si el cálculo da 65536, al pasarlo a uint16 se vuelve 0.
    // Lo fuerzo a quedarse en 65535.
    if (raw_pwm > 65535) raw_pwm = 65535;

    // PWM (La librería espera uint16)
    if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, (uint16_t)raw_pwm);

    // Incremento el contador para el siguiente ciclo (10ms después)
    iden_counter++;
}

*/