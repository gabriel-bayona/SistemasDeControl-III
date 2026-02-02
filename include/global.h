#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <ESP32_FastPWM.h>
#include <ESP32Encoder.h>

// Objetos globales
extern ESP32_FAST_PWM* pwm;
extern ESP32Encoder encoder;
extern TaskHandle_t TaskPID_Handle;

// Variables COMPARTIDAS (Volatile para acceso Dual Core)
extern volatile double kp, ki, kd;
extern volatile double setpoint;
extern volatile double posicion_actual;
extern volatile int32_t duty_global; // 32 bits para evitar overflow antes del cast
extern volatile bool b_PID;
extern volatile bool b_space_states_controlled; // Bandera para control por espacio de estados
extern volatile bool b_windup;
extern volatile bool b_control_pot; // Bandera para habilitar control por potenciómetro


// Variables de Espacio de Estados
extern double k_x1; // Ganancia de Posición
extern double k_x2; // Ganancia de Velocidad
extern double k0;   // Ganancia de Pre-compensación (Nbar)


//Banderas y variables no críticas
extern bool b_logging; // Bandera para activar/desactivar logging
extern volatile bool b_identification; // Bandera para activar modo identificación
extern volatile bool b_identification2; // Bandera para activar modo identificación 2
extern volatile int iden_counter;      // Contador de pasos (ticks de 10ms)

void initPeripherals(); //Inicializa el hardware necesario

#endif