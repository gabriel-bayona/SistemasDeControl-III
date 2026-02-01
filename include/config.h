#ifndef CONFIG_H
#define CONFIG_H

// CONFIGURACIÓN DE PINES
#define PWM_PIN 17
#define ENABLE 5
#define SENTIDO_HORARIO 16
#define SENTIDO_ANTIHORARIO 15
#define CHA_A 12
#define CHA_B 14
#define POT_PIN 34 // Pin ADC para el potenciómetro

// CONFIGURACIÓN PWM Y PID
#define PWM_CHANNEL 0
#define RESOLUTION 10       // Resolución de Hardware (10 bits)  - no es un pin del esp32
#define FRECUENCIA 5000     // Hz del PWM
#define MAX_VALUE_ENCODER 62300.0 //lo pongo float para que el computo de divisiones sea en float
#define V_PID_MAX 12.0 // idem encoder

#endif