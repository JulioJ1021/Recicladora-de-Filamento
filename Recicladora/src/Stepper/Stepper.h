#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

void Stepper_Init();
void Stepper_ON();
void Stepper_OFF();
bool Stepper_isRunning();

#endif