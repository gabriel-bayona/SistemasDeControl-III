#ifndef MENU_H
#define MENU_H

#include <Arduino.h> 

/* Función principal, maneja el menú por Serial. El menú permite controlar el sistema vía comandos por el monitor serial.*/
void handleSerialMenu();

// Prototipos de funciones de comando, permite llamar desde cualquier parte del código

/*Función que imprime la ayuda por Serial. Se muestran los comandos disponibles para controlar el sistema*/
void printHelp();

/*Función que habilita el PID clásico. Recordar que debe establecerse los valores de las constantes kp, ki, kd; de no hacerlo, el sistema toma valores default.*/
void cmd_EnablePID();

/*Función que habilita el control por Espacio de Estados. Recordar que debe establecerse los valores de las ganancias del controlador; de no hacerlo, el sistema toma valores default.*/
void cmd_EnableSS();

/*Función que detiene el motor. Deshabilita las bandas de control. */
void cmd_Stop();

/*Función que activa/desactiva el control del setpoint por potenciómetro - Normalizado 0 - 1. Por defecto inicia desactivado.*/
void cmd_TogglePot();

/*Función que establece el setpoint a un valor dado. El setpoint debe estar normalizado, tomando el valor 1 como una vuelta horaria. Permite valores negativos y cero.*/
void cmd_SetSetpoint(String inputStr);

/*Función que actualiza las constantes del PID clásico(kp, ki, kd).*/
void cmd_UpdatePID(String inputStr);

/*Función que actualiza las ganancias del controlador por Espacio de Estados ()*/
void cmd_UpdateSS(String inputStr);

/*Función que establece setpoint y constantes del PID clásico*/
void cmd_SetAll(String inputStr);

/*Función que inicia el modo de identificación - "1": Rampas, "2": Escalones */
void cmd_StartIdentification(int mode);

/*Función que muestra el estado actual del sistema*/
void cmd_Debug();

/*Función que activa/desactiva el logging por serial. El formato mostrado es "Time,Setpoint,Pos,Error,PWM"*/
void cmd_ToggleLog();

#endif