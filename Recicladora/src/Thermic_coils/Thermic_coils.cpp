
#include "Thermic_coils.h"

#include <max6675.h>
#include <math.h>
#include "esp_timer.h"

// ============================================================
// 1. CONFIGURACION
// ============================================================

// MAX6675 - Zona 1
#define TC1_SCK 18
#define TC1_CS  5
#define TC1_SO  19

// MAX6675 - Zona 2
#define TC2_SCK 18
#define TC2_CS  17
#define TC2_SO  16

// SSR
#define SSR1_PIN 2
#define SSR2_PIN 15

// Tiempos
#define TM_SSR             500000UL
#define MAX_TEMP_C         270.0f
#define TIMER_PERIOD_US    10000ULL
#define TIMER_TICKS_WINDOW 500U

// ============================================================
// 2. VARIABLES INTERNAS
// ============================================================

static MAX6675 thermocouple1(TC1_SCK, TC1_CS, TC1_SO);
static MAX6675 thermocouple2(TC2_SCK, TC2_CS, TC2_SO);

static hw_timer_t *timerVentana = NULL;
static esp_timer_handle_t calcTimer = NULL;

static portMUX_TYPE ventanaMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint16_t windowTick = 0;
static volatile uint16_t dutyTicks1 = 0;
static volatile uint16_t dutyTicks2 = 0;

// Instancias públicas
ControlSSR SSR1 = {};
ControlSSR SSR2 = {};

// ============================================================
// 3. CONFIGURACION DE CADA ZONA
// ============================================================

static void configurarZona(
    ControlSSR &ssr,
    MAX6675 *sensor,
    uint8_t pin,
    float setpoint,
    float kp,
    float ki,
    float kd
) {
    ssr.sensor = sensor;
    ssr.ssrPin = pin;

    ssr.tempRaw = 0.0f;
    ssr.tempEMA = 0.0f;
    ssr.alphaTemp = 0.3f;
    ssr.tempEmaInit = false;

    ssr.sp = setpoint;
    ssr.kp = kp;
    ssr.ki = ki;
    ssr.kd = kd;
    ssr.tm = TM_SSR / 1000000.0f;

    ssr.error = 0.0f;
    ssr.error1 = 0.0f;
    ssr.error2 = 0.0f;

    ssr.cv = 0.0f;
    ssr.cv1 = 0.0f;
    ssr.cvEMA = 0.0f;
    ssr.alphaCV = 0.2f;
    ssr.cvEmaInit = false;

    ssr.duty = 0.0f;
}

// ============================================================
// 4. VENTANA DE CONTROL SSR
// ============================================================

static void actualizarDutyVentana(float duty1, float duty2) {

    uint16_t ticks1 = (uint16_t)(
        constrain(duty1, 0.0f, 100.0f)
        * TIMER_TICKS_WINDOW / 100.0f
    );

    uint16_t ticks2 = (uint16_t)(
        constrain(duty2, 0.0f, 100.0f)
        * TIMER_TICKS_WINDOW / 100.0f
    );

    portENTER_CRITICAL(&ventanaMux);

    dutyTicks1 = ticks1;
    dutyTicks2 = ticks2;

    portEXIT_CRITICAL(&ventanaMux);
}

// ============================================================
// 5. INTERRUPCION DEL TIMER SSR
// ============================================================

static void ARDUINO_ISR_ATTR onVentanaTimer() {

    bool estadoSSR1;
    bool estadoSSR2;

    portENTER_CRITICAL_ISR(&ventanaMux);

    windowTick++;

    if (windowTick >= TIMER_TICKS_WINDOW) {
        windowTick = 0;
    }

    estadoSSR1 = (windowTick < dutyTicks1);
    estadoSSR2 = (windowTick < dutyTicks2);

    portEXIT_CRITICAL_ISR(&ventanaMux);

    digitalWrite(SSR1_PIN, estadoSSR1 ? HIGH : LOW);
    digitalWrite(SSR2_PIN, estadoSSR2 ? HIGH : LOW);
}

static void ventanaTimer_Setup() {

    timerVentana = timerBegin(1000000);

    timerAttachInterrupt(
        timerVentana,
        &onVentanaTimer
    );

    timerAlarm(
        timerVentana,
        TIMER_PERIOD_US,
        true,
        0
    );
}

// ============================================================
// 6. LECTURA DE TEMPERATURA
// ============================================================

static float getTemp(ControlSSR &ssr) {

    if (ssr.sensor == nullptr) {
        return NAN;
    }

    float temp = ssr.sensor->readCelsius();

    ssr.tempRaw = temp;

    return temp;
}

// ============================================================
// 7. CONTROL PID Y FILTROS EMA
// ============================================================

static void tempControl(ControlSSR &ssr) {

    if (isnan(ssr.tempRaw)) {

        ssr.duty = 0.0f;

        actualizarDutyVentana(
            SSR1.duty,
            SSR2.duty
        );

        return;
    }

    // FILTRO EMA DE TEMPERATURA

    if (!ssr.tempEmaInit) {

        ssr.tempEMA = ssr.tempRaw;
        ssr.tempEmaInit = true;

    } else {

        ssr.tempEMA =
            ssr.alphaTemp * ssr.tempRaw +
            (1.0f - ssr.alphaTemp) * ssr.tempEMA;
    }

    // SETPOINT Y ERROR

    ssr.error = ssr.sp - ssr.tempEMA;

    // ECUACION DE DIFERENCIAS

    ssr.cv = ssr.cv1
           + (ssr.kp + ssr.kd / ssr.tm) * ssr.error
           + (-ssr.kp + ssr.ki * ssr.tm
           - 2.0f * ssr.kd / ssr.tm) * ssr.error1
           + (ssr.kd / ssr.tm) * ssr.error2;

    ssr.cv = constrain(ssr.cv, 0.0f, 100.0f);

    // FILTRO EMA DE LA SALIDA

    if (!ssr.cvEmaInit) {

        ssr.cvEMA = ssr.cv;
        ssr.cvEmaInit = true;

    } else {

        ssr.cvEMA =
            ssr.alphaCV * ssr.cv +
            (1.0f - ssr.alphaCV) * ssr.cvEMA;
    }

    // SEGURIDAD

    if (ssr.tempRaw > MAX_TEMP_C || ssr.error < -20.0f) {

        ssr.cvEMA = 0.0f;
    }

    // ACTUALIZAR ESTADOS

    ssr.cv1 = ssr.cv;
    ssr.error2 = ssr.error1;
    ssr.error1 = ssr.error;

    // DUTY FINAL

    ssr.duty = constrain(ssr.cvEMA, 0.0f, 100.0f);

    actualizarDutyVentana(
        SSR1.duty,
        SSR2.duty
    );
}

// ============================================================
// 8. CALLBACK DE CONTROL TERMICO
// ============================================================

static void ssrCallback(void *args) {

    getTemp(SSR1);
    getTemp(SSR2);

    tempControl(SSR1);
    tempControl(SSR2);
}

// ============================================================
// 9. TEMPORIZADOR DE CONTROL
// ============================================================

static void ssrTimer_Setup() {

    esp_timer_create_args_t args = {};

    args.callback = &ssrCallback;
    args.name = "SSRcontrol";
    args.dispatch_method = ESP_TIMER_TASK;

    ESP_ERROR_CHECK(
        esp_timer_create(&args, &calcTimer)
    );

    ESP_ERROR_CHECK(
        esp_timer_start_periodic(calcTimer, TM_SSR)
    );
}

// ============================================================
// 10. FUNCIONES PUBLICAS
// ============================================================

void thermalSetup() {

    // Configuración de las zonas

    configurarZona(
        SSR1,
        &thermocouple1,
        SSR1_PIN,
        0.0f,
        1.4171f,
        0.01079f,
        0.0f
    );

    configurarZona(
        SSR2,
        &thermocouple2,
        SSR2_PIN,
        0.0f,
        1.4171f,
        0.01079f,
        0.0f
    );

    // Salidas SSR

    pinMode(SSR1_PIN, OUTPUT);
    pinMode(SSR2_PIN, OUTPUT);

    digitalWrite(SSR1_PIN, LOW);
    digitalWrite(SSR2_PIN, LOW);

    // Inicialización de temporizadores

    ventanaTimer_Setup();
    ssrTimer_Setup();
}

void setSP(ControlSSR &ssr, float SP) {

    ssr.sp = SP;
}

void fijarDuty(ControlSSR &ssr, float nuevo_duty) {

    ssr.duty = constrain(nuevo_duty, 0.0f, 100.0f);

    actualizarDutyVentana(
        SSR1.duty,
        SSR2.duty
    );
}

void imprimirEstadoSSR(const ControlSSR &ssr, uint8_t zonaId) {

    Serial.printf(
        "SSR%u | Raw: %.2f C | EMA: %.2f C | Duty: %.1f%%\n",
        zonaId,
        ssr.tempRaw,
        ssr.tempEMA,
        ssr.duty
    );
}
