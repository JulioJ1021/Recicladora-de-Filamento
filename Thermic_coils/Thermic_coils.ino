#include <Arduino.h>
#include <max6675.h>
#include <math.h>
#include "esp_timer.h"

// ============================================================
// 1. PINES
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

// ============================================================
// 2. CONFIGURACION DE TIEMPOS Y VENTANA
// ============================================================

#define TM_SSR             500000UL // 500000 us = 0.5 s (muestreo y control)
#define MAX_TEMP_C         270.0f

#define TIMER_PERIOD_US    10000ULL // 10 ms por tick de ventana
#define TIMER_TICKS_WINDOW 500U     // 500 * 10 ms = 5000 ms (5 s)

hw_timer_t *timerVentana = NULL;
static portMUX_TYPE ventanaMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t windowTick = 0;
volatile uint16_t dutyTicks1 = 0;
volatile uint16_t dutyTicks2 = 0;

// ============================================================
// 3. SENSORES Y ESTRUCTURA DE CONTROL SSR
// ============================================================

MAX6675 thermocouple1(TC1_SCK, TC1_CS, TC1_SO);
MAX6675 thermocouple2(TC2_SCK, TC2_CS, TC2_SO);

struct ControlSSR {
  MAX6675 *sensor;      // Puntero a la termocupla asociada a esta zona
  uint8_t ssrPin;       // Pin del SSR de esta zona

  float tempRaw;        // Lectura directa de la termocupla (°C)
  float tempEMA;        // Temperatura filtrada por EMA (°C)
  float alphaTemp;      // Factor EMA para temperatura (0.0 - 1.0)
  bool tempEmaInit;     // Primera muestra de temperatura ya cargada

  float sp;             // Setpoint (°C)
  float kp;
  float ki;
  float kd;
  float tm;             // Tiempo de muestreo en segundos

  float error;
  float error1;
  float error2;

  float cv;             // Salida cruda del PID (0.0 - 100.0 %)
  float cv1;            // Salida anterior del PID
  float cvEMA;          // Salida del PID filtrada por EMA (0.0 - 100.0 %)
  float alphaCV;        // Factor EMA para salida PID (0.0 - 1.0)
  bool cvEmaInit;       // Primera muestra de CV ya cargada

  float duty;           // Duty final listo para la ventana del SSR (0.0 - 100.0 %)
};

ControlSSR SSR1 = {
  .sensor      = &thermocouple1,
  .ssrPin      = SSR1_PIN,

  .tempRaw     = 0.0f,
  .tempEMA     = 0.0f,
  .alphaTemp   = 0.3f,
  .tempEmaInit = false,

  .sp          = 120.0f,
  .kp          = 2.0f,
  .ki          = 0.8f,
  .kd          = 0.0f,
  .tm          = TM_SSR / 1000000.0f,

  .error       = 0.0f,
  .error1      = 0.0f,
  .error2      = 0.0f,

  .cv          = 0.0f,
  .cv1         = 0.0f,
  .cvEMA       = 0.0f,
  .alphaCV     = 0.2f,
  .cvEmaInit   = false,

  .duty        = 0.0f
};

ControlSSR SSR2 = {
  .sensor      = &thermocouple2,
  .ssrPin      = SSR2_PIN,

  .tempRaw     = 0.0f,
  .tempEMA     = 0.0f,
  .alphaTemp   = 0.3f,
  .tempEmaInit = false,

  .sp          = 60.0f,
  .kp          = 2.0f,
  .ki          = 0.5f,
  .kd          = 0.0f,
  .tm          = TM_SSR / 1000000.0f,

  .error       = 0.0f,
  .error1      = 0.0f,
  .error2      = 0.0f,

  .cv          = 0.0f,
  .cv1         = 0.0f,
  .cvEMA       = 0.0f,
  .alphaCV     = 0.2f,
  .cvEmaInit   = false,

  .duty        = 0.0f
};

// ============================================================
// 4. INTERRUPCION DE VENTANA PARA SSR (CADA 10 ms)
// ============================================================

void ARDUINO_ISR_ATTR onVentanaTimer() {
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

void ventanaTimer_Setup() {
  timerVentana = timerBegin(1000000); // 1 MHz -> 1 us por tick
  timerAttachInterrupt(timerVentana, &onVentanaTimer);
  timerAlarm(timerVentana, TIMER_PERIOD_US, true, 0);
}

// ============================================================
// 5. DUTY, LECTURA Y CONTROL PI
// ============================================================

void actualizarDutyVentana(float duty1, float duty2) {
  uint16_t ticks1 = (uint16_t)(
    constrain(duty1, 0.0f, 100.0f) * TIMER_TICKS_WINDOW / 100.0f
  );
  uint16_t ticks2 = (uint16_t)(
    constrain(duty2, 0.0f, 100.0f) * TIMER_TICKS_WINDOW / 100.0f
  );

  portENTER_CRITICAL(&ventanaMux);
  dutyTicks1 = ticks1;
  dutyTicks2 = ticks2;
  portEXIT_CRITICAL(&ventanaMux);
}

void fijarDuty(ControlSSR &ssr, float nuevo_duty) {
  ssr.duty = constrain(nuevo_duty, 0.0f, 100.0f);
  actualizarDutyVentana(SSR1.duty, SSR2.duty);
}

float getTemp(ControlSSR &ssr) {
  if (ssr.sensor == nullptr) return NAN;
  float temp = ssr.sensor->readCelsius();
  ssr.tempRaw = temp;
  return temp;
}

void tempControl(ControlSSR &ssr) {
  if (isnan(ssr.tempRaw)) {
    ssr.duty = 0.0f;
    actualizarDutyVentana(SSR1.duty, SSR2.duty);
    Serial.println("ayayay");
    return;
  }

  // ---- FILTRO EMA DE TEMPERATURA ----
  if (!ssr.tempEmaInit) {
    ssr.tempEMA = ssr.tempRaw;
    ssr.tempEmaInit = true;
  } else {
    ssr.tempEMA = ssr.alphaTemp * ssr.tempRaw +
                  (1.0f - ssr.alphaTemp) * ssr.tempEMA;
  }

  // ---- SET POINT Y ERROR ----
  ssr.error = ssr.sp - ssr.tempEMA;

  // ---- ECUACIÓN DE DIFERENCIAS ----
  ssr.cv = ssr.cv1
         + (ssr.kp + ssr.kd / ssr.tm) * ssr.error
         + (-ssr.kp + ssr.ki * ssr.tm - 2.0f * ssr.kd / ssr.tm) * ssr.error1
         + (ssr.kd / ssr.tm) * ssr.error2;

  ssr.cv = constrain(ssr.cv, 0.0f, 100.0f);

  // ---- FILTRO EMA DE LA SALIDA ----
  if (!ssr.cvEmaInit) {
    ssr.cvEMA = ssr.cv;
    ssr.cvEmaInit = true;
  } else {
    ssr.cvEMA = ssr.alphaCV * ssr.cv +
                (1.0f - ssr.alphaCV) * ssr.cvEMA;
  }

  // ---- SEGURIDAD ----
  if (ssr.tempRaw > MAX_TEMP_C || ssr.error < -20.0f) {
    ssr.cvEMA = 0.0f;
  }

  // ---- ACTUALIZAR ESTADOS ----
  ssr.cv1 = ssr.cv;
  ssr.error2 = ssr.error1;
  ssr.error1 = ssr.error;

  // ---- DUTY FINAL ----
  ssr.duty = constrain(ssr.cvEMA, 0.0f, 100.0f);
  actualizarDutyVentana(SSR1.duty, SSR2.duty);
}

void ssrCallback(void *args) {
  getTemp(SSR1);
  getTemp(SSR2);
  tempControl(SSR1);
  tempControl(SSR2);
}

void ssrTimer_Setup() {
  esp_timer_handle_t calcTimer = NULL;
  esp_timer_create_args_t args = {};
  args.callback        = &ssrCallback;
  args.name            = "SSRcontrol";
  args.dispatch_method = ESP_TIMER_TASK;
  ESP_ERROR_CHECK(esp_timer_create(&args, &calcTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(calcTimer, TM_SSR));
}

void setSP(ControlSSR &ssr, float SP) {
  ssr.sp = SP;
}

void imprimirEstadoSSR(const ControlSSR &ssr, uint8_t zonaId) {
  Serial.printf("SSR%u | Raw: %.2f C | EMA: %.2f C | Duty: %.1f%%\n",
                zonaId, ssr.tempRaw, ssr.tempEMA, ssr.duty);
}
// ============================================================
// 6. SETUP Y LOOP
// ============================================================

void thermalSetup(){
  pinMode(SSR1_PIN, OUTPUT);
  pinMode(SSR2_PIN, OUTPUT);
  digitalWrite(SSR1_PIN, LOW);
  digitalWrite(SSR2_PIN, LOW);
  ventanaTimer_Setup();
  ssrTimer_Setup();
}

void setup() {
  Serial.begin(115200);
  thermalSetup();
  delay(500); 
}

void loop() {
  Serial.print("T1: ");
  Serial.print(SSR1.tempRaw);
  Serial.print(" C (EMA: ");
  Serial.print(SSR1.tempEMA);
  Serial.print(") Duty1: ");
  Serial.print(SSR1.duty);
  Serial.print("% || T2: ");
  Serial.print(SSR2.tempRaw);
  Serial.print(" C (EMA: ");
  Serial.print(SSR2.tempEMA);
  Serial.print(") Duty2: ");
  Serial.print(SSR2.duty);
  Serial.println("%");
  delay(500);
}