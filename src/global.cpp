#include "global.h"
#include "config.h"

// Inicialización de objetos
ESP32_FAST_PWM* pwm = nullptr; // Puntero para inicialización en setup. Seguridad para evitar escribir en memoria basura
ESP32Encoder encoder;
TaskHandle_t TaskPID_Handle; // Handle de la tarea PID

// Inicialización de variables (para no tener problemas de basura y demás)
volatile double kp = 4.5, ki = 0.6, kd = 0.0; // esto va a depender del sistema, lo dejo así para pruebas
volatile double setpoint = 0.0;
volatile double posicion_actual = 0.0;
volatile int32_t duty_global = 0;
volatile bool b_PID = false;
volatile bool b_windup = true;
bool b_logging = false; // Inicializada en falso para no loggear hasta que se pida
volatile bool b_identification = false; //idem arriba
volatile bool b_identification2 = false; //idem arriba
volatile int iden_counter = 0;
volatile bool b_control_pot = false; // Inicializado en falso por seguridad

// Variables de Espacio de Estados
volatile bool b_space_states_controlled = false; // Inicializado en falso. "false": PID clásico ; "true": Espacio de Estados
double k_x1 = 100.0; 
double k_x2 = 5.0;   
double k0 = 100.0;


void initPeripherals() {
    // Configuración de Pines
    pinMode(ENABLE, OUTPUT);
    pinMode(SENTIDO_HORARIO, OUTPUT);
    pinMode(SENTIDO_ANTIHORARIO, OUTPUT);
    digitalWrite(ENABLE, HIGH); // Driver habilitado siempre para este caso

    // Configuración del Encoder
    encoder.attachFullQuad(CHA_A, CHA_B);
    encoder.clearCount();

    // Configuración segura del PWM
    pwm = new ESP32_FAST_PWM(PWM_PIN, FRECUENCIA, 0, PWM_CHANNEL, RESOLUTION);
    
    if (!pwm) {
        Serial.println("ERROR FATAL: No se pudo iniciar el PWM");
        while(1); // Bloqueo infinito de seguridad
    }
}