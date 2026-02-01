#include "PID_Task.h"
#include "config.h"
#include "global.h"
#include "identification.h"


// Función de la tarea PID (Núcleo 0)
void TaskPIDCode( void * pvParameters ){
  // Variables locales del PID (para no saturar el bus de memoria)
  double error_actual, error_previo = 0, integral = 0, derivativo = 0;
  double output_u;
  const double Ts = 0.01; // 10ms
  
  double max_output_percent = 65535.0 * 0.9; // Salida máxima en escala PWM de 16 bits (corresponde a V_PID_MAX)

  // Bucle infinito de la tarea (Reemplaza al Timer0)
  for(;;){

    //LEER DATOS
    long counts = encoder.getCount();
    posicion_actual = (double)counts / MAX_VALUE_ENCODER; // Normalizo a 0.0 - 1.0 (1 vuelta completa)

    if(b_PID){  

      if(b_control_pot) {
          // El ESP32 tiene ADC de 12 bits (0 a 4095)
          int lect_adc = analogRead(POT_PIN);
          
          // Convertir a rango 0.0 a 1.0
          
          setpoint = (double)lect_adc / 4095.0; 
          //Serial.printf("Lectura Potenciómetro ADC: %d -> Setpoint: %.3f\n", lect_adc, setpoint);
          
      }
      
      //CÁLCULO DE ERROR
      error_actual = setpoint - posicion_actual;
      
      //PID
      // Proporcional
      double P = kp * error_actual;
      
      // Integral (con anti-windup básico: solo integra si no saturamos)
      if(output_u < V_PID_MAX && output_u > -V_PID_MAX){
         integral += ki * error_actual * Ts;
      }
      
      // Derivativo
      derivativo = kd * (error_actual - error_previo) / Ts;
      
      // Salida final (expresada en tension)
      output_u = P + integral + derivativo;
      error_previo = error_actual;

      // LOGICA DE SALIDA Y PROTECCION DE DESBORDAMIENTO
      // (0-12V) a escala PWM de 16 bits (0-65535)
      //long long para el cálculo intermedio para evitar desbordamiento
      long long calc_duty = (long long)((output_u / V_PID_MAX) * 65535.0);
      
      // Mapeo de signo y dirección
      if(calc_duty >= 0){
          digitalWrite(SENTIDO_HORARIO, HIGH);
          digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      } else {
          digitalWrite(SENTIDO_HORARIO, LOW);
          digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
          calc_duty = -calc_duty; 
      }

      // Si el cálculo da 65536, al pasarlo a uint16 se vuelve 0.
      // Lo fuerzo a quedarse en 65535.
      if (calc_duty > max_output_percent ) calc_duty= max_output_percent;
      
      duty_global = (int16_t)calc_duty; //  para debug

      // PWM (La librería espera uint16)
      pwm->setPWM_Int(PWM_PIN, FRECUENCIA, duty_global);
      
    } 
    else if (b_identification) {
        runIdentificationStep(); 
    }
    else if (b_identification2) {
        runIdentificationStep2(); 
    }
    else {
      //PID apagado, motor quieto
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      integral = 0; // Reset integral
      error_previo = 0;
      output_u = 0;
    }
    
    // Muestreo preciso de 10ms (100Hz)
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}