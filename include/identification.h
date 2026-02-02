#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <Arduino.h>

// Esta función ejecutará un paso del experimento.
// Debe llamarse cada 10ms desde la Tarea.

/*runIdentificationStep realiza un paso del experimento de identificación por rampas*/
void runIdentificationStep();

/*runIdentificationStep2 realiza un paso del experimento de identificación por escalones*/
void runIdentificationStep2();

#endif