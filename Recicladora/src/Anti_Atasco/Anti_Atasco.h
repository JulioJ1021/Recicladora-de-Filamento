#ifndef ANTI_ATASCO_H
#define ANTI_ATASCO_H

#include <Arduino.h>

void Anti_Atasco_Setup();

void activarMotores();
void apagarMotores();
bool Anti_Atasco_isRecovering();

#endif