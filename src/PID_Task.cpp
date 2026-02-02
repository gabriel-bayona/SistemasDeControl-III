#include "PID_Task.h"
#include "config.h"
#include "global.h"
#include "identification.h"

static const double Ts = 0.01;   // Periodo de muestreo: 10 ms


double getSetpoint(double current_sp) {
  // Si el control por potenciómetro está habilitado, leer el ADC  
  if (b_control_pot) {
        int lect_adc = analogRead(POT_PIN);
        // El ESP32 tiene ADC de 12 bits (0-4095)
        return (double)lect_adc / 4095.0;
    }
    return current_sp;
}


void velocityFilterInit(VelocityFilter *f) {
    for (int i = 0; i < 5; i++) f->buffer[i] = 0;
}
double velocityFilterUpdate(VelocityFilter *filtro_vel, double vel_inst) {
    // Desplazamiento del buffer
    for (int i = 0; i < 4; i++){
        filtro_vel->buffer[i] = filtro_vel->buffer[i + 1];
    }

    filtro_vel->buffer[4] = vel_inst;

    // Cálculo del promedio
    double sum = 0;
    for (int i = 0; i < 5; i++){
        sum += filtro_vel->buffer[i];
    }
    return sum / 5.0;
}

void pidInit(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_output = 0;       // u_1
    pid->prev_encoder_counts = 0;
}

void pidReset(PIDController *pid) {
    pid->integral = 0;
    pid->prev_output = 0;
    pid->prev_error = 0;
}


double pidDigitalUpdate(PIDController *pid, double setpoint, long current_counts, VelocityFilter *f) {
    
    // 1. Cálculo de constantes discretas 
    double a = pid->kp + pid->ki * Ts;
    double b = pid->ki * Ts;
    double c = pid->kd / Ts;

    // 2. Cálculo del Error
    double pos_norm = (double)current_counts / MAX_VALUE_ENCODER;
    double eT = setpoint - pos_norm;

    // 3. Lógica de Anti-windup Personalizada
    // "Solo acumula si no está saturado o si el error ayuda a des-saturar"
    // Usamos pid->prev_output como u_1
    bool cond_saturacion = ((pid->prev_output < V_PID_MAX) || (eT < 0)) && 
                           ((pid->prev_output > -V_PID_MAX) || (eT > 0));

    // Si b_windup es true, habilita la protección.
    if ((!b_windup) || cond_saturacion) {
        pid->integral = b * eT + pid->integral; // iT = b*eT + iT0
    }

    // 4. Término Derivativo (Usando filtro de deltas crudos)
    long delta_counts = current_counts - pid->prev_encoder_counts;
    pid->prev_encoder_counts = current_counts; // Actualizar memoria
    
    // Usamos el filtro existente pero pasándole el delta crudo
    double delta_filtrado = velocityFilterUpdate(f, (double)delta_counts);

    // Fórmula: d(k) = c * [delta_filtrado] / MAX_VALUE
    double dT = c * delta_filtrado / MAX_VALUE_ENCODER;

    // 5. Señal de control total
    // u(k) = a*e(k) + i(k) + d(k)
    double uT = (a * eT) + pid->integral + dT;

    // 6. Saturación de Salida
    if (uT > V_PID_MAX) uT = V_PID_MAX;
    else if (uT < -V_PID_MAX) uT = -V_PID_MAX;
 
    // 7. Actualizar u_1 para el siguiente ciclo
    pid->prev_output = uT;

    return uT;
}


double pidClassicUpdate(PIDController *pid, double error) {
    // Término Proporcional
    double kp_aux = pid->kp * error;

    // Término Integral con Anti-windup
    double integral_tentativa = pid->integral + (pid->ki * error * Ts);
    
    // Si la integral tentativa excede el máximo, la saturamos (Anti-windup)
    if (integral_tentativa > V_PID_MAX) {
        pid->integral = V_PID_MAX;
    } else if (integral_tentativa < -V_PID_MAX) {
        pid->integral = -V_PID_MAX;
    } else {
        pid->integral = integral_tentativa;
    }

    // Término Derivativo
    double kd_aux = pid->kd * (error - pid->prev_error) / Ts;
    pid->prev_error = error;

    return kp_aux + pid->integral + kd_aux;
}

void writeMotorOutput(double u_volts) {
    const double max_duty = 65535.0 * 0.9; // Límite de seguridad del PWM (90%)

    // Saturación de tension (Simulada)
    if (u_volts > V_PID_MAX)  u_volts = V_PID_MAX;
    if (u_volts < -V_PID_MAX) u_volts = -V_PID_MAX;

    // Conversión de tensión a Duty Cycle (16 bits)
    long long duty = (long long)((u_volts / V_PID_MAX) * 65535.0);

    // Control de Dirección (Puente H)
    if (duty >= 0) {
        digitalWrite(SENTIDO_HORARIO, HIGH);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
    } else {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
        duty = -duty; // Tomar valor absoluto para el PWM
    }

    // Saturación final del contador PWM
    if (duty > max_duty) duty = max_duty;

    duty_global = (int16_t)duty; // Variable global para debug
    pwm->setPWM_Int(PWM_PIN, FRECUENCIA, duty_global);
}


void TaskPIDCode(void *pvParameters) {

    // Inicialización de estructuras locales
    PIDController pid;
    pidInit(&pid, kp, ki, kd);

    // Filtro A: Para Espacio de Estados (Velocidad normalizada)
    VelocityFilter vel_filter_ss;
    velocityFilterInit(&vel_filter_ss);

    // Filtro B: Exclusivo para PID Digital (Deltas crudos)
    VelocityFilter vel_filter_digital;
    velocityFilterInit(&vel_filter_digital);

    double pos_prev = 0; // Memoria para derivada de posición

    for (;;) {

        long counts = encoder.getCount();
        posicion_actual = (double)counts / MAX_VALUE_ENCODER;
        double vel_inst = (posicion_actual - pos_prev) / Ts;
        pos_prev = posicion_actual;
        double vel_x2 = velocityFilterUpdate(&vel_filter_ss, vel_inst);
        setpoint = getSetpoint(setpoint);

        if (b_PID) {
            //Actualizar constantes desde globales por si cambiaron via Serial (spid)
            pid.kp = kp;
            pid.ki = ki;
            pid.kd = kd;

            double error = setpoint - posicion_actual;
            //double u = pidClassicUpdate(&pid, error);
            double u = pidDigitalUpdate(&pid, setpoint, counts, &vel_filter_digital);

            writeMotorOutput(u);
        }
        else if (b_space_states_controlled) {
            // Reseteamos PID para evitar "patada" si volvemos a cambiar
            pidReset(&pid);
            
            // Ley de control: u = -K*x + N_bar*r
            double u = (k0 * setpoint) - (k_x1 * posicion_actual) - (k_x2 * vel_x2);
            
            writeMotorOutput(u);
        }
        else if (b_identification) {
            runIdentificationStep();
        }
        else if (b_identification2) {
            runIdentificationStep2();
        }
        else {
            writeMotorOutput(0);
            pidReset(&pid);
            
            pid.prev_encoder_counts = counts;

            // Reiniciar filtros y estados para arranque suave
            pos_prev = posicion_actual; 
            velocityFilterInit(&vel_filter_ss);
            velocityFilterInit(&vel_filter_digital);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
