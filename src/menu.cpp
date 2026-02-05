#include "menu.h"
#include "global.h" // variables compartidas
#include "config.h" // constantes de configuración

void printHelp() {
  Serial.println("Comandos disponibles:");
  Serial.println("  help          : Muestra esta ayuda");
  Serial.println("  epid          : Inicia el control PID");
  Serial.println("  stop          : Detiene el motor");
  Serial.println("  setp <valor>  : Establece el setpoint (valor en unidades de posición, 1 unidad = 1 vuelta completa)");
  Serial.println("  set pot       : Activa/Desactiva control por potenciómetro (0 a 1.0)");
  Serial.println("  spid <kp> <kd> <ki> : Actualiza las constantes PID clasico");
  Serial.println("  setall <setpoint> <kp> <ki> <kd> : Establece setpoint y constantes PID clasico");
  Serial.println("  ess           : Inicia el control por Espacio de Estados - Enable State Space");
  Serial.println("  sss <k0> <kx1> <kx2> : Actualiza las ganancias del controlador por Espacio de Estados");
  Serial.println("  iden          : Inicia el modo de identificación (rampas de duty cíclicas)");
  Serial.println("  iden2         : Inicia el modo de identificación 2 (escalones de duty cíclicos)");
  Serial.println("  iden dz     : Inicia el modo de identificación para DEADZONE, sirve para encontrar la tension mínima de arranque");
  Serial.println("  log           : Activa/Desactiva el logging de datos por serial");
  Serial.println("  renc      : Resetea el contador del encoder a 0");
  Serial.println("  debug         : Muestra estado actual del sistema");
  Serial.println("");
  Serial.println("Ejemplo: setp 0.5");
  Serial.println("         spid 20.0 5.0 0.1");
  Serial.println("         setall 0.7 15.0 0.0 2.0");
  Serial.println("         sss 120.0 80.0 10.0");
}

void cmd_EnablePID() {
  encoder.clearCount();
  b_PID = true;
  b_space_states_controlled = false; // Asegurar exclusión mutua
  g_iden_mode = IDEN_NONE; // Asegurar que no estamos en modo identificación
  setpoint = posicion_actual; // arranque suave
  Serial.println("PID Activado.");
}

void cmd_EnableSS() {
  b_PID = false;  // Asegurar que PID está apagado
  g_iden_mode = IDEN_NONE; // Asegurar que no estamos en modo identificación

  b_space_states_controlled = true;    // Prender SS
  
  //setpoint = posicion_actual; // arranque suave
  Serial.println("Modo: Espacio de Estados Activado.");
}

void cmd_Stop() {
  b_PID = false;
  b_space_states_controlled = false;
  duty_applied = 0;
  g_iden_mode = IDEN_NONE;
  b_logging = false;
  b_control_pot = false;
  // Es buena práctica asegurar que el PWM físico vaya a 0 aquí también por seguridad
  pwm->setPWM_Int(PWM_PIN, FRECUENCIA, duty_applied);
  Serial.println("Motor Detenido.");
}

void cmd_resetEncoder() {
  encoder.clearCount();
  Serial.println("Encoder reseteado a 0.");
}

void cmd_TogglePot() {
  b_control_pot = !b_control_pot; // Invierte el estado
  if (b_control_pot) {
      Serial.println("Control por Potenciometro: ACTIVADO (0 a 1.0)");
  } else {
      Serial.println("Control por Potenciometro: DESACTIVADO");
  }
}

void cmd_SetSetpoint(String inputStr) {
  b_control_pot = false; // asegurarse de desactivar control por potenciómetro
  inputStr.remove(0, 4); // Remover "setp"
  setpoint = inputStr.toDouble();
  Serial.printf("Nuevo Setpoint: %.2f\n", setpoint);
}

void cmd_UpdatePID(String inputStr) {
  float proportional_in, derivative_in, integral_in;
  // Parseo de la cadena completa "spid <kp> <kd> <ki>"
  if(sscanf(inputStr.c_str(), "spid %f %f %f", &proportional_in, &derivative_in, &integral_in) == 3) {
      kp = proportional_in; kd = derivative_in; ki = integral_in;
      Serial.printf("PID Update: %.2f %.2f %.2f\n", kp, kd, ki);
  } else {
      Serial.println("Error formato PID. Usa: spid <kp> <kd> <ki>");
  }
}

void cmd_UpdateSS(String inputStr) {
  float k_0_in, k_x1_in, k_x2_in;
  // Corregido: sscanf debe buscar "sss ..."
  if(sscanf(inputStr.c_str(), "sss %f %f %f", &k_0_in, &k_x1_in, &k_x2_in) == 3){
    k0 = k_0_in;
    k_x1 = k_x1_in;
    k_x2 = k_x2_in;
    Serial.printf("State Space Update -> K0: %.2f | Kx1: %.2f | Kx2: %.2f\n", k0, k_x1, k_x2);
  } else {
    Serial.println("Error formato SS. Usa: sss <k0> <kx1> <kx2>");
  }
}

void cmd_SetAll(String inputStr) {
  float setpoint_in, proportional_in, integral_in, derivative_in;
  if(sscanf(inputStr.c_str(), "setall %f %f %f %f", &setpoint_in, &proportional_in, &integral_in, &derivative_in) == 4) {
      setpoint = setpoint_in;
      kp = proportional_in;
      ki = integral_in;
      kd = derivative_in;

      Serial.printf("SET ALL -> Valores asignados:");
      Serial.printf("  Setpoint: %.3f\n", setpoint);
      Serial.printf("  kp:       %.3f\n", kp);
      Serial.printf("  ki:       %.3f\n", ki);
      Serial.printf("  kd:       %.3f\n", kd);
  } else {
      Serial.println("Error formato. Usa: setall <sp> <kp> <ki> <kd>");
  }
}

void cmd_StartIdentification(int mode) {
  b_PID = false;          // Apagar PID
  b_space_states_controlled = false; // Apagar SS
  
  iden_counter = 0;       // Reset contador
  b_logging = true;       // Activar logging automáticamente
  encoder.clearCount();

  if (mode == 1) {
    g_iden_mode = IDEN_RAMP;
    Serial.println("INICIO IDENTIFICACION 1 - RAMPAS");
  } else if (mode == 2) {
    g_iden_mode = IDEN_STEP;
    Serial.println("INICIO IDENTIFICACION 2 - ESCALONES");
  }else if (mode == 3) {
    g_iden_mode = IDEN_DEADZONE;
    Serial.println("INICIO IDENTIFICACION 3 - POSIBLE ZONA MUERTA - BUSCA TENSION MINIMA DE ARRANQUE");
  } else {
    Serial.println("Modo de identificación inválido.");
    return;
  }
  Serial.println("Time,Pos,PWM_Duty"); // Header
}

void cmd_ToggleLog() {
  b_logging = !b_logging;
  if (b_logging) Serial.println("Time,Setpoint,Pos,Error,PWM"); 
  else Serial.println("Log detenido.");
}

void cmd_Debug() {
   Serial.printf("Modo PID: %d | Modo SS: %d |Pos: %.3f | Set: %.2f | Duty Raw: %d | Duty %%: %d\n", 
                 b_PID, b_space_states_controlled, posicion_actual, setpoint, duty_applied, (duty_applied * 100 / 65535));
}


void handleSerialMenu() {
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Eliminar espacios en el inicio y final
    
    printf("Comando recibido: '%s'\n", inputStr.c_str());

    if (inputStr == "help")         printHelp();  
    else if (inputStr == "epid")    cmd_EnablePID();
    else if (inputStr == "ess")     cmd_EnableSS();
    else if (inputStr == "stop")    cmd_Stop();
    else if (inputStr == "iden")    cmd_StartIdentification(1);
    else if (inputStr == "iden2")   cmd_StartIdentification(2);
    else if (inputStr == "log")     cmd_ToggleLog();
    else if (inputStr == "debug")   cmd_Debug();
    else if (inputStr == "renc")    cmd_resetEncoder();

    
    // Comandos compuestos (startsWith)
    else if (inputStr.startsWith("set pot")) cmd_TogglePot();
    else if (inputStr.startsWith("setp"))    cmd_SetSetpoint(inputStr);
    else if (inputStr.startsWith("spid"))    cmd_UpdatePID(inputStr);
    else if (inputStr.startsWith("setall"))  cmd_SetAll(inputStr);
    else if (inputStr.startsWith("sss"))     cmd_UpdateSS(inputStr);
    else if (inputStr.startsWith("iden dz"))   cmd_StartIdentification(3);

    // Comando no reconocido
    else Serial.println("Comando no reconocido. Escribe 'help' para ver los comandos disponibles.");
  }
}