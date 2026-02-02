#ifndef CONFIG_H
#define CONFIG_H

// CONFIGURACIÓN DE PINES
#define PWM_PIN 17 // Pin de salida PWM desde el microcontrolador al Driver.
#define ENABLE 5        // Pin de Enable desde el microcontrolador para el pin del Driver (Activo Alto).
#define SENTIDO_HORARIO 16 // Pin de control de sentido horario desde el microcontrolador al Driver.
#define SENTIDO_ANTIHORARIO 15 // Pin de control de sentido antihorario desde el microcontrolador al Driver.
#define CHA_A 12      // Pin A del Encoder
#define CHA_B 14     // Pin B del Encoder
#define POT_PIN 34 // Pin ADC para el potenciómetro

// CONFIGURACIÓN PWM Y PID
#define PWM_CHANNEL 0
#define RESOLUTION 10       // Resolución de Hardware (10 bits)  - no es un pin del esp32
#define FRECUENCIA 5000     // Hz del PWM
#define MAX_VALUE_ENCODER 62300.0 //Cantidad de cuentas del encoder por vuetas. Lo pongo float para que el computo de divisiones sea en float.
#define V_PID_MAX 12.0 // Tension máxima de salida del PID en Volts.

#endif