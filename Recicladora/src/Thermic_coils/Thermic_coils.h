
#ifndef THERMIC_COILS_H
#define THERMIC_COILS_H

#include <Arduino.h>

class MAX6675;

// ============================================================
// ESTRUCTURA DE CONTROL DE CADA ZONA
// ============================================================

struct ControlSSR {

    MAX6675 *sensor;
    uint8_t ssrPin;

    float tempRaw;
    float tempEMA;
    float alphaTemp;
    bool tempEmaInit;

    float sp;
    float kp;
    float ki;
    float kd;
    float tm;

    float error;
    float error1;
    float error2;

    float cv;
    float cv1;
    float cvEMA;
    float alphaCV;
    bool cvEmaInit;

    float duty;
};

// ============================================================
// INSTANCIAS DE LAS ZONAS
// ============================================================

extern ControlSSR SSR1;
extern ControlSSR SSR2;

// ============================================================
// FUNCIONES PUBLICAS
// ============================================================

// Inicialización del módulo
void thermalSetup();

// Modificación del setpoint
void setSP(ControlSSR &ssr, float SP);

// Modificación manual del duty
void fijarDuty(ControlSSR &ssr, float nuevo_duty);

// Impresión de variables
void imprimirEstadoSSR(const ControlSSR &ssr, uint8_t zonaId);

#endif
