#include "menu.h"
#include "global.h" // variables compartidas

void handleSerialMenu() {
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr == "help")
    {
      Serial.println("Comandos disponibles:");
      Serial.println("  help          : Muestra esta ayuda");
      Serial.println("  epid          : Inicia el control PID");
      Serial.println("  stop          : Detiene el motor");
      Serial.println("  setp <valor>  : Establece el setpoint (valor en unidades de posición, 1 unidad = 1 vuelta completa)");
      Serial.println("  set pot       : Activa/Desactiva control por potenciómetro (0 a 1.0)");
      Serial.println("  spid <kp> <kd> <ki> : Actualiza las constantes PID");
      Serial.println("  setall <setpoint> <kp> <ki> <kd> : Establece setpoint y constantes PID");
      Serial.println("  iden          : Inicia el modo de identificación (rampas de duty cíclicas)");
      Serial.println("  iden2         : Inicia el modo de identificación 2 (escalones de duty cíclicos)");
      Serial.println("  log           : Activa/Desactiva el logging de datos por serial");
      Serial.println("  debug         : Muestra estado actual del sistema");
      Serial.println("");
      Serial.println("Ejemplo: setp 0.5");
      Serial.println("         spid 20.0 5.0 0.1");
      Serial.println("         setall 0.7 15.0 0.0 2.0");
      
    }
    
    else if (inputStr == "epid") {
      encoder.clearCount();
      b_PID = true;
      Serial.println("PID Activado.");
    }
    else if (inputStr == "stop") {
      b_PID = false;
      Serial.println("Motor Detenido.");
    }
    else if (inputStr.startsWith("set pot")) {
    b_control_pot = !b_control_pot; // Invierte el estado

      if (b_control_pot) {
          Serial.println("Control por Potenciometro: ACTIVADO (0 a 1.0)");
      } else {
          Serial.println("Control por Potenciometro: DESACTIVADO");
          
      }
    }

    else if (inputStr.startsWith("setp")) {
      b_control_pot = false; // asegurarse de desactivar control por potenciómetro
      inputStr.remove(0, 4);
      setpoint = inputStr.toDouble();
      Serial.printf("Nuevo Setpoint: %.2f\n", setpoint);
    }
    else if (inputStr.startsWith("spid")) {
        // Parseo simple para actualizar constantes al vuelo
        float p, d, i;
        sscanf(inputStr.c_str(), "spid %f %f %f", &p, &d, &i);
        kp = p; kd = d; ki = i;
        Serial.printf("PID Update: %.2f %.2f %.2f\n", kp, kd, ki);
      }
      else if (inputStr.startsWith("setall")) {
      float sp, p, i, d;
      sscanf(inputStr.c_str(), "setall %f %f %f %f", &sp, &p, &i, &d);

      setpoint = sp;
      kp = p;
      ki = i;
      kd = d;

      Serial.printf("SET ALL -> Valores asignados:");
      Serial.printf("  Setpoint: %.3f\n", setpoint);
      Serial.printf("  kp:       %.3f\n", kp);
      Serial.printf("  ki:       %.3f\n", ki);
      Serial.printf("  kd:       %.3f\n", kd);

  }
  else if (inputStr == "iden") {
      b_PID = false;          // Apagar PID para que no interfiera
      b_identification = true; // Activar modo iden
      iden_counter = 0;       // Reset contador
      b_logging = true;    // Activar logging para guardar datos de identificación
      encoder.clearCount();
      
      Serial.println("INICIO IDENTIFICACION 1 - RAMPAS");
      Serial.println("Time,Pos,PWM_Duty"); // Header compatible con Matlab
  }
  else if (inputStr == "iden2") {
      b_PID = false;          // Apagar PID para que no interfiera
      b_identification2 = true; // Activar modo iden2
      iden_counter = 0;       // Reset contador
      b_logging = true;    // Activar logging para guardar datos de identificación
      encoder.clearCount();
      
      Serial.println("INICIO IDENTIFICACION 2 - ESCALONES");
      Serial.println("Time,Pos,PWM_Duty"); // Header compatible con Matlab
  }
    else if (inputStr == "log") {
        b_logging = !b_logging;
        if (b_logging) Serial.println("Time,Setpoint,Pos,Error,PWM"); // Header
        else Serial.println("Log detenido.");
    }


    else if (inputStr == "debug") {
       Serial.printf("Pos: %.3f | Set: %.2f | Duty Raw: %d | Duty %: %d\n", posicion_actual, setpoint, duty_global, (duty_global * 100 / 65535));
    }
  }
}