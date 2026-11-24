
#ifndef CONTROL_MOTORES_H
#define CONTROL_MOTORES_H

#include <Arduino.h>

// ============================================================
// INICIALIZACION
// ============================================================

void Motor_control_Setup();

// ============================================================
// SETPOINTS
// ============================================================

void setSP_Puller(float setpoint);
void setSP_Extrusor(float setpoint);

// ============================================================
// LECTURAS
// ============================================================

float getRPM_Puller();
float getRPM_Extrusor();

float getCV_Puller();
float getCV_Extrusor();

#endif
