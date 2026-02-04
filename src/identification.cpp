#include "identification.h"
#include "global.h"
#include "config.h"
#include "PID_Task.h"

// Funciones internas (archivo-local)
static void runIdentificationRamp();
static void runIdentificationStep();


static void runIdentificationRamp() {
    if (g_iden_mode != IDEN_RAMP) return;

    float duty_percent = 0.0;
    float max_duty_percent = 90.0; // Máximo duty cycle en porcentaje
    
    // Definimos la duración de cada "diente" del triángulo en ticks del contador
    // Ejemplo: 0 ms a 300 ms sube, 300 ms a 600 ms baja.

    uint32_t t_subida  = IDEN_RAMP_SAMPLES;
    uint32_t t_bajada  = 2 * IDEN_RAMP_SAMPLES; // Punto donde termina la bajada (Ej si IDEN_RAMP_SAMPLES = 3 [s], 300 [ms] de subida + 300 [ms] de bajada)
    uint32_t t_cambio  = 2 * IDEN_RAMP_SAMPLES; // Punto donde cambiamos de sentido
    
    // PRIMERA ETAPA: SENTIDO ANTIHORARIO (0 a 600 ticks)
    if (iden_counter < t_cambio) {

        if (iden_counter < t_subida) {
            // Rampa de subida (0% a max%)
            // Ecuación lineal: y = (x / x_max) * y_max
            duty_percent = ((float)iden_counter / (float)t_subida) * max_duty_percent;
        } else {
            // Rampa de bajada (max% a 0%)
            // Ecuación: mapeamos de t_subida->t_bajada a max->0
            float pasos_en_bajada = iden_counter - t_subida;
            float duracion_bajada = t_bajada - t_subida;
            duty_percent = max_duty_percent * (1.0 - (pasos_en_bajada / duracion_bajada));
        }
        
    // SEGUNDA ETAPA: SENTIDO HORARIO (600 a 1200 ticks)
    } else if (iden_counter >= t_cambio && iden_counter < (t_cambio * 2)) {

        int local_counter = iden_counter - t_cambio; // Reiniciamos contador virtual para esta etapa

        if (local_counter < t_subida) {
            // Rampa subida
            duty_percent = ((float)local_counter / (float)t_subida) * max_duty_percent;
        } else {
            // Rampa bajada
            float pasos_en_bajada = local_counter - t_subida;
            float duracion_bajada = t_bajada - t_subida;
            duty_percent = -max_duty_percent * (1.0 - (pasos_en_bajada / duracion_bajada));
        }

    // FINALIZACIÓN
    } else {

        if(pwm) pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
        
        duty_applied = 0;
        g_iden_mode = IDEN_NONE; // Salimos del modo identificación
        b_logging = false; 
        iden_counter = 0; // Reseteamos para la próxima vez
        Serial.println("FIN IDENTIFICACION");
        return; 
    }

    double u_volts = (duty_percent / 100.0) * V_PID_MAX;
    
    writeMotorOutput(u_volts);

    iden_counter++;

}


static void runIdentificationStep() {
    if (g_iden_mode != IDEN_STEP) return;
        
    uint32_t current_step = iden_counter / IDEN2_STEP_SAMPLES;

    if (current_step >= IDEN2_TOTAL_STEPS) {

        writeMotorOutput(0.0);

        g_iden_mode = IDEN_NONE; // Salimos del modo identificación
        b_logging = false;
        iden_counter = 0;

        Serial.println("FIN IDENTIFICACION 2");
        return;
    }

    // Alternar signo: + - + - ..., hace que los escalones sean positivos y negativos, cambia el sentido del motor
    double sign = (current_step % 2 == 0) ? 1.0 : -1.0;

    // Aplicar el escalón correspondiente, alternando el signo
    double u_volts = sign * (IDEN2_MAX_DUTY_PCT / 100.0) * V_PID_MAX;

    writeMotorOutput(u_volts);

    iden_counter++;
}


void runIdentification()
{
    switch (g_iden_mode) {

        case IDEN_RAMP:
            runIdentificationRamp();
            break;

        case IDEN_STEP:
            runIdentificationStep();
            break;

        case IDEN_NONE:
        default:
            printf("\nrunIdentification: Error, se ingreso a un modo de identificación no válido.\n");
            break;
    }
}




/*
void runIdentificationRamp() {
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

    duty_applied = (int32_t)duty;
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