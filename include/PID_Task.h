#ifndef PID_TASK_H
#define PID_TASK_H

#include <Arduino.h> // Acá está el FreeRTOS, ya tiene su ifndefs internos

/**
 * @brief Estructura para el Filtro de Media Móvil de Velocidad.
 * @details Implementa un buffer circular FIFO de 5 muestras para suavizar
 * el ruido inherente a la derivada numérica de la posición.
 */
typedef struct {
    double buffer[5]; ///< Buffer de almacenamiento de muestras de velocidad
} VelocityFilter;

/**
 * @brief Estructura de control PID.
 * @details Contiene los parámetros de sintonización y la memoria del integrador/derivativo.
 * Se separa el estado (memoria) de la configuración para permitir múltiples instancias.
 */
typedef struct {
    // Parámetros de configuración
    double kp;          ///< Ganancia Proporcional
    double ki;          ///< Ganancia Integral
    double kd;          ///< Ganancia Derivativa
    
    // Variables de estado (Memoria)
    double integral;    ///< Acumulador del término integral - Equivale a iT0
    double prev_output;       ///< Equivale a u_1 (Salida anterior)
    double prev_error;          // e(k-1) -> Para calcular derivada clásica
    long prev_encoder_counts; ///< Para calcular delta de posición crudo
} PIDController;

/**
    * @brief Función que inicializa el filtro de media móvil de velocidad.
    * @param f Puntero a la estructura VelocityFilter a inicializar.
 */
void velocityFilterInit(VelocityFilter *f);

/**
 * @brief Función que actualiza el filtro de media móvil de velocidad.  
 * 
 * @param f Puntero a la estructura VelocityFilter.
 * @param vel_inst Velocidad instantánea a agregar al filtro.
 * @return double Velocidad promediada.
 */
double velocityFilterUpdate(VelocityFilter *f, double vel_inst);

/**
 * @brief   Inicializa la estructura del controlador PID clasico con los parámetros dados.
 * 
 * @param pid Puntero a la estructura PIDController a inicializar.
 * @param kp Ganancia proporcional. Variable volatile global.
 * @param ki Ganancia integral. Variable volatile global.
 * @param kd Ganancia derivativa. Variable volatile global.
 */
void pidInit(PIDController *pid, double kp, double ki, double kd);

/**
 * @brief   Resetea los valores de estado del controlador PID (integral y error previo).
 * 
 * @param pid Puntero a la estructura PIDController a resetear.
 */
void pidReset(PIDController *pid);


double pidDigitalUpdate(PIDController *pid, double setpoint, long current_counts, VelocityFilter *f);


/**
 * @brief   Calcula la salida del controlador PID clásico dado el error actual.
 * 
 * @param pid Estructura del controlador PID.
 * @param error Error actual (referencia - estado actual).
 * @return double Salida del controlador PID (kp * error + ki * integral + kd * derivada).
 */
double pidClassicUpdate(PIDController *pid, double error);

/**
 * @brief Obtiene el setpoint actual, leyendo el potenciómetro si está habilitado para actuar de controlador del setpoint.  
 * 
 * @param current_sp Setpoint actual.
 * @return double setpoint actualizado.
 */
double getSetpoint(double current_sp);

/**
 * @brief Actualiza la salida del motor, mapeando la tensión de control a PWM y dirección.
 * 
 * @param u_volts Tensión de control calculada por el controlador (en Volts).
 */
void writeMotorOutput(double u_volts);

/**
 * @brief Esta función es la tarea que corre en el Núcleo 0 y ejecuta el control PID. 
 * 
 * Contiene dos modos: PID clásico y Espacio de Estados.
 * 
 * Diseñada con el patrón "Read -> Process -> Write". 
 *  Lógica separada en 4 bloques claros:
 * 
 * 1. Estimación: Calcular posición y velocidad (x_1, x_2).
 * 
 * 2. Referencia: Leer el potenciómetro o mantener el valor serial.
 * 
 * 3. Algoritmo: Calcular la tensión de control u(t) (PID Clásico o Espacio de Estados).
 * 
 * 4. Actuación: Mover el hardware (PWM y Pines de dirección).
 * 
 * Cada bloque es independiente y modular, facilitando futuras modificaciones.
 * @param pvParameters 
 */
void TaskPIDCode(void * pvParameters);


#endif