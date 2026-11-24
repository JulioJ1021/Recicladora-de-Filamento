// ============================================================
// RECICLADORA DE FILAMENTO - SKETCH UNICO (ESP32)
// Generado a partir de Recicladora.ino + src/*
// ============================================================

#include <Arduino.h>
#include <max6675.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_err.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>   // libreria "WebSockets"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ---- Thermic_coils (declaraciones) ----
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

// ---- Control_Motores (declaraciones) ----
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

// ---- Post_Extrusion (declaraciones) ----
void Post_Extrusion_Setup();
void Diam_timer_Setup();
float getDiameter();
bool getCoolingState();
void setEmbobinando(bool state);

// ---- Stepper (declaraciones) ----
void Stepper_Init();
void Stepper_ON();
void Stepper_OFF();
bool Stepper_isRunning();

// ---- Anti_Atasco (declaraciones) ----
void Anti_Atasco_Setup();

void activarMotores();
void apagarMotores();
bool Anti_Atasco_isRecovering();

// ---- HMI_WiFi (declaraciones) ----
// ============================================================
// CONFIGURACION  (EDITAR)
// ============================================================

#define HMI_WIFI_SSID      "PIPECAJICA"
#define HMI_WIFI_PASSWORD  "dafe1961"

// Nombre en la red: la HMI abre ws://recicladora.local:81
#define HMI_MDNS_NAME      "recicladora"
#define HMI_WS_PORT        81

// ============================================================
// DATOS QUE SE ENVIAN A LA HMI
// ============================================================

struct HMIDatos {

    float mica1;
    float mica2;

    float rpm_ext;
    float rpm_puller;

    float diametro;
    float spool_m;

    bool running;
    bool estop;

    bool tritu;
    bool extru;
    bool enfri;
    bool embo;
};

enum HMIComando {
    HMI_START,
    HMI_STOP,
    HMI_BOBINAR
};

// ============================================================
// API  (el servidor corre en una tarea propia en el Core 0)
// ============================================================

// Llamar una vez en setup()
void HMI_Setup();

// Core 1: publicar la ultima lectura de las variables
void HMI_publicarDatos(const HMIDatos &datos);

// Core 1: sacar un comando recibido de la HMI (false si no hay)
bool HMI_leerComando(HMIComando &cmd, uint8_t &cliente);

// Core 1: responder el resultado de un comando a la HMI
void HMI_responder(uint8_t cliente, const char *cmd, bool ok, const char *reason);

// ============================================================
// MODULO: Thermic_coils
// ============================================================
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
static esp_timer_handle_t calcTimer_term = NULL;

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
        esp_timer_create(&args, &calcTimer_term)
    );

    ESP_ERROR_CHECK(
        esp_timer_start_periodic(calcTimer_term, TM_SSR)
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

// ============================================================
// MODULO: Control_Motores
// ============================================================
// ============================================================
// 1. CONFIGURACION
// ============================================================

#define TIMEOUT_PULSOS_US 500000
#define TIMER1_PERIOD     50000

#define SENSOR_DIAM 36

#define FREQ_Puller   34
#define FREQ_Extrusor 39

#define PWM_Puller   21
#define PWM_Extrusor 25

#define PWM_FREQ  70000
#define PWM_RESOL 10

#define EMA_ALPHA    0.2f
#define EMA_CV_ALPHA 0.2f

#define PULSES_PER_REV_Puller 11.0f
#define GEAR_RATIO_Puller     21.3f

// ============================================================
// 2. PARAMETROS PID
// ============================================================

// PULLER

static float sp_puller = 0.0f;

static float pv_puller = 0.0f;

static float cv_puller = 0.0f;
static float cv1_puller = 0.0f;

static float error_puller = 0.0f;
static float error1_puller = 0.0f;
static float error2_puller = 0.0f;

static float Kp_puller = 1.4410f;
static float Ki_puller = 4.5246f;
static float Kd_puller = 0.000f;

// EXTRUSOR

static float sp_extrusor = 0.0f;

static float pv_extrusor = 0.0f;

static float cv_extrusor = 0.0f;
static float cv1_extrusor = 0.0f;

static float error_extrusor = 0.0f;
static float error1_extrusor = 0.0f;
static float error2_extrusor = 0.0f;

static float Kp_extrusor = 2.5705f;
static float Ki_extrusor = 2.8041f;
static float Kd_extrusor = 0.000f;

// Tiempo de muestreo

static float Tm = TIMER1_PERIOD / 1000000.0f;

// ============================================================
// 3. TEMPORIZADOR
// ============================================================

static const uint64_t CALC_PERIOD_US = TIMER1_PERIOD;

// ============================================================
// 4. DATOS DE CAPTURA MCPWM
// ============================================================

typedef struct {

    uint32_t last_cap;

    volatile uint32_t period;

    volatile bool valid;

    bool first;

    volatile uint32_t last_pulse_us;

} cap_data_t;

static cap_data_t data_a = {0, 0, false, false, 0};
static cap_data_t data_b = {0, 0, false, false, 0};

static uint32_t res_hz = 0;

// Handles MCPWM

static mcpwm_cap_timer_handle_t cap_timer = NULL;

static mcpwm_cap_channel_handle_t chan_a = NULL;
static mcpwm_cap_channel_handle_t chan_b = NULL;

// Handle del temporizador de cálculo

static esp_timer_handle_t calcTimer_motores = NULL;

// ============================================================
// 5. VARIABLES RPM
// ============================================================

// RPM

static volatile float rpm_puller = 0.0f;
static volatile float rpm_extrusor = 0.0f;

// RPM EMA

static volatile float rpmEMA_puller = 0.0f;
static volatile float rpmEMA_extrusor = 0.0f;

static bool emaInitialized_puller = false;
static bool emaInitialized_extrusor = false;

// CV EMA

static volatile float cvEMA_puller = 0.0f;
static volatile float cvEMA_extrusor = 0.0f;

static bool cvEmaInitialized_puller = false;
static bool cvEmaInitialized_extrusor = false;

// ============================================================
// 6. CALLBACK DE CAPTURA MCPWM
// ============================================================

static bool IRAM_ATTR cap_callback(
    mcpwm_cap_channel_handle_t chan,
    const mcpwm_capture_event_data_t *edata,
    void *user_data
) {

    cap_data_t *d = (cap_data_t *)user_data;

    d->last_pulse_us = (uint32_t)esp_timer_get_time();

    if (d->first) {

        d->period = edata->cap_value - d->last_cap;

        d->valid = true;
    }

    d->last_cap = edata->cap_value;

    d->first = true;

    return false;
}

// ============================================================
// 7. CONFIGURACION DE CANALES MCPWM
// ============================================================

static void setup_channel(
    mcpwm_cap_timer_handle_t timer,
    int gpio,
    cap_data_t *data,
    mcpwm_cap_channel_handle_t *channel
) {

    mcpwm_capture_channel_config_t cfg = {};

    cfg.gpio_num = gpio;

    cfg.prescale = 1;

    cfg.flags.pos_edge = true;
    cfg.flags.neg_edge = false;

    cfg.flags.pull_up = false;
    cfg.flags.pull_down = false;

    ESP_ERROR_CHECK(
        mcpwm_new_capture_channel(timer, &cfg, channel)
    );

    mcpwm_capture_event_callbacks_t cbs = {};

    cbs.on_cap = cap_callback;

    ESP_ERROR_CHECK(
        mcpwm_capture_channel_register_event_callbacks(
            *channel,
            &cbs,
            data
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_capture_channel_enable(*channel)
    );
}

// ============================================================
// 8. PWM
// ============================================================

static void setPWM(uint8_t pin, uint8_t duty) {

    int pwm = constrain(
        duty * 1023.0f / 100.0f,
        0.0f,
        1023.0f
    );

    ledcWrite(pin, pwm);
}

// ============================================================
// 9. CALCULO DE RPM Y PID
// ============================================================

static void onTimer1(void *arg) {

    float freq_a = 0.0f;
    float freq_b = 0.0f;

    uint32_t ahora_us = (uint32_t)esp_timer_get_time();

    // --------------------------------------------------------
    // RPM PULLER
    // --------------------------------------------------------

    if (data_a.valid && data_a.period > 0) {

        freq_a = (float)res_hz / (float)data_a.period;

        rpm_puller = freq_a * 60.0f /
            (PULSES_PER_REV_Puller * GEAR_RATIO_Puller);

        if (!emaInitialized_puller) {

            rpmEMA_puller = rpm_puller;

            emaInitialized_puller = true;

            cvEmaInitialized_puller = true;

        } else {

            rpmEMA_puller =
                EMA_ALPHA * rpm_puller +
                (1.0f - EMA_ALPHA) * rpmEMA_puller;
        }
    }

    // --------------------------------------------------------
    // RPM EXTRUSOR
    // --------------------------------------------------------

    if (data_b.valid && data_b.period > 0) {

        freq_b = (float)res_hz / (float)data_b.period;

        rpm_extrusor = freq_b * 60.0f /
            (PULSES_PER_REV_Puller * GEAR_RATIO_Puller);

        if (!emaInitialized_extrusor) {

            rpmEMA_extrusor = rpm_extrusor;

            emaInitialized_extrusor = true;

            cvEmaInitialized_extrusor = true;

        } else {

            rpmEMA_extrusor =
                EMA_ALPHA * rpm_extrusor +
                (1.0f - EMA_ALPHA) * rpmEMA_extrusor;
        }
    }

    // --------------------------------------------------------
    // TIMEOUT PULLER
    // --------------------------------------------------------

    if ((uint32_t)(ahora_us - data_a.last_pulse_us)
        > TIMEOUT_PULSOS_US) {

        rpm_puller = 0.0f;
        rpmEMA_puller = 0.0f;
    }

    // --------------------------------------------------------
    // TIMEOUT EXTRUSOR
    // --------------------------------------------------------

    if ((uint32_t)(ahora_us - data_b.last_pulse_us)
        > TIMEOUT_PULSOS_US) {

        rpm_extrusor = 0.0f;
        rpmEMA_extrusor = 0.0f;
    }

    // --------------------------------------------------------
    // REINICIO DE BANDERAS
    // --------------------------------------------------------

    data_a.valid = false;
    data_b.valid = false;

    // --------------------------------------------------------
    // MONITOREO
    // --------------------------------------------------------

    /*Serial.printf(
        "GPIO%d: %.2f RPM | GPIO%d: %.2f RPM\n",
        FREQ_Puller,
        rpmEMA_puller,
        FREQ_Extrusor,
        rpm_extrusor
    );*/

    // ========================================================
    // PID PULLER
    // ========================================================

    error_puller = sp_puller - rpmEMA_puller;

    cv_puller = cv1_puller
        + (Kp_puller + Kd_puller / Tm) * error_puller
        + (-Kp_puller + Ki_puller * Tm
        - 2 * Kd_puller / Tm) * error1_puller
        + (Kd_puller / Tm) * error2_puller;

    cv_puller = constrain(cv_puller, 0.0f, 100.0f);

    // EMA de salida

    if (!cvEmaInitialized_puller) {

        cvEMA_puller = cv_puller;

        cvEmaInitialized_puller = true;

    } else {

        cvEMA_puller =
            EMA_CV_ALPHA * cv_puller +
            (1.0f - EMA_CV_ALPHA) * cvEMA_puller;
    }

    // Actualizar estados

    cv1_puller = cv_puller;

    error2_puller = error1_puller;
    error1_puller = error_puller;

    setPWM(PWM_Puller, cvEMA_puller);

    // ========================================================
    // PID EXTRUSOR
    // ========================================================

    error_extrusor = sp_extrusor - rpmEMA_extrusor;

    cv_extrusor = cv1_extrusor
        + (Kp_extrusor + Kd_extrusor / Tm) * error_extrusor
        + (-Kp_extrusor + Ki_extrusor * Tm
        - 2 * Kd_extrusor / Tm) * error1_extrusor
        + (Kd_extrusor / Tm) * error2_extrusor;

    cv_extrusor = constrain(cv_extrusor, 0.0f, 100.0f);

    // EMA de salida

    if (!cvEmaInitialized_extrusor) {

        cvEMA_extrusor = cv_extrusor;

        cvEmaInitialized_extrusor = true;

    } else {

        cvEMA_extrusor =
            EMA_CV_ALPHA * cv_extrusor +
            (1.0f - EMA_CV_ALPHA) * cvEMA_extrusor;
    }

    // Actualizar estados

    cv1_extrusor = cv_extrusor;

    error2_extrusor = error1_extrusor;
    error1_extrusor = error_extrusor;

    setPWM(PWM_Extrusor, cvEMA_extrusor);
}

// ============================================================
// 10. TEMPORIZADOR DE CALCULO
// ============================================================

static void setupCalcTimer() {

    esp_timer_create_args_t args = {};

    args.callback = &onTimer1;

    args.name = "rpm_calc";

    args.dispatch_method = ESP_TIMER_TASK;

    ESP_ERROR_CHECK(
        esp_timer_create(&args, &calcTimer_motores)
    );

    ESP_ERROR_CHECK(
        esp_timer_start_periodic(calcTimer_motores, CALC_PERIOD_US)
    );
}

// ============================================================
// 11. INICIALIZACION DEL MODULO
// ============================================================

void Motor_control_Setup() {

    // Timer de captura

    mcpwm_capture_timer_config_t timer_cfg = {};

    timer_cfg.group_id = 0;

    timer_cfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;

    ESP_ERROR_CHECK(
        mcpwm_new_capture_timer(&timer_cfg, &cap_timer)
    );

    ESP_ERROR_CHECK(
        mcpwm_capture_timer_get_resolution(
            cap_timer,
            &res_hz
        )
    );

    Serial.printf(
        "Resolucion del timer de captura: %lu Hz\n",
        (unsigned long)res_hz
    );

    // Canales de captura

    setup_channel(
        cap_timer,
        FREQ_Puller,
        &data_a,
        &chan_a
    );

    setup_channel(
        cap_timer,
        FREQ_Extrusor,
        &data_b,
        &chan_b
    );

    // Activar captura

    ESP_ERROR_CHECK(
        mcpwm_capture_timer_enable(cap_timer)
    );

    ESP_ERROR_CHECK(
        mcpwm_capture_timer_start(cap_timer)
    );

    // Configurar PWM

    ledcAttach(PWM_Extrusor, PWM_FREQ, PWM_RESOL);

    ledcAttach(PWM_Puller, PWM_FREQ, PWM_RESOL);

    // Iniciar cálculo periódico

    setupCalcTimer();
}

// ============================================================
// 12. FUNCIONES PUBLICAS
// ============================================================

void setSP_Puller(float setpoint) {

    sp_puller = setpoint;
}

void setSP_Extrusor(float setpoint) {

    sp_extrusor = setpoint;
}

float getRPM_Puller() {

    return rpmEMA_puller;
}

float getRPM_Extrusor() {

    return rpmEMA_extrusor;
}

float getCV_Puller() {

    return cvEMA_puller;
}

float getCV_Extrusor() {

    return cvEMA_extrusor;
}

// ============================================================
// MODULO: Post_Extrusion
// ============================================================
// ============================================================
// CONFIGURACIÓN
// ============================================================

#define DIAM_SENS       36
#define VENT            23
#define M_EMB           4

#define UMBRAL_DIAM     10
#define TM_DIAM         50000

#define FREQ_PWM_EMB    20000
#define RES_PWM_EMB     10
#define DUTY_EMB        512

// ============================================================
// VARIABLES INTERNAS
// ============================================================

static bool ENFRIANDO = false;
static bool EMBOBINANDO = false;

static float diameter = 0.0f;
static float rpm_puller_pe = 0.0f;

static esp_timer_handle_t diameter_timer = NULL;

// ============================================================
// FUNCIONES INTERNAS
// ============================================================

static float diam2rpms(float diam) {

    float rpms = 2.0f * diam - 0; // Función temporal

    return rpms;
}

static float diam_calc(int sensor_read) {

    float diam = sensor_read / 8.0f;

    return diam;
}

// ============================================================
// GESTIÓN DEL DIÁMETRO
// ============================================================

static void Diam_manage(void *arg) {

    diameter = diam_calc(analogRead(DIAM_SENS));

    ENFRIANDO = (diameter > UMBRAL_DIAM) ? true : false;

    if (ENFRIANDO) {

        GPIO.out_w1ts = (1UL << VENT);

        

        Serial.println("Frio");

        rpm_puller_pe = diam2rpms(diameter);

        // Setpoint RPM acorde a RPM del puller
        // Considerar PID para el diámetro del filamento

    } else {

        GPIO.out_w1tc = (1UL << VENT);

        //ledcWrite(M_EMB, 0);

        // Setpoint RPM en 0
    }
}

// ============================================================
// CONFIGURACIÓN GPIO Y PWM
// ============================================================

void Post_Extrusion_Setup() {

    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask = (1ULL << VENT);

    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ledcAttach(M_EMB, FREQ_PWM_EMB, RES_PWM_EMB);

    ledcWrite(M_EMB, 0);

    analogReadResolution(10);
}

// ============================================================
// CONFIGURACIÓN DEL TEMPORIZADOR
// ============================================================

void Diam_timer_Setup() {

    const esp_timer_create_args_t timer_args = {

        .callback = &Diam_manage,

        .arg = NULL,

        .dispatch_method = ESP_TIMER_TASK,

        .name = "Diameter sensor"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &diameter_timer));

    ESP_ERROR_CHECK(
        esp_timer_start_periodic(diameter_timer, TM_DIAM)
    );
}

float getDiameter() {
    return diameter;
}

bool getCoolingState() {
    return ENFRIANDO;
}

void setEmbobinando(bool state) {
    EMBOBINANDO = state;

    if (EMBOBINANDO) {
        ledcWrite(M_EMB, DUTY_EMB);
    } else {
        ledcWrite(M_EMB, 0);
    }
}

// ============================================================
// MODULO: Stepper
// ============================================================
// ============================================================
// CONFIGURACIÓN
// ============================================================

#define STEP_RPMS       10
#define PASOS_X_VUELTA  4096

#define ST1 26
#define ST2 27
#define ST3 14
#define ST4 13

// ============================================================
// VARIABLES INTERNAS
// ============================================================

static const int step_buffer[4] = {ST1, ST2, ST3, ST4};

static int step_counter = 0;
static esp_timer_handle_t step_timer = NULL;

// ============================================================
// CALLBACK DEL TEMPORIZADOR
// ============================================================

static void Step_Timer_callback(void* arg) {

    GPIO.out_w1tc = (1ULL << step_buffer[step_counter]);

    step_counter++;
    step_counter &= 3;

    GPIO.out_w1ts =
        (1UL << step_buffer[step_counter]) |
        (1UL << step_buffer[(step_counter + 1) & 3]);

}

// ============================================================
// CONFIGURACIÓN DEL TEMPORIZADOR
// ============================================================

static void Step_Timer_Setup() {

    const esp_timer_create_args_t timer_args = {
        .callback = &Step_Timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "timer_motor_pasos"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &step_timer));
}

// ============================================================
// CONFIGURACIÓN DE GPIO
// ============================================================

static void Step_setup() {

    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask =
        (1ULL << ST1) |
        (1ULL << ST2) |
        (1ULL << ST3) |
        (1ULL << ST4);

    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

// ============================================================
// INICIALIZACIÓN DEL MÓDULO
// ============================================================

void Stepper_Init() {

    Step_Timer_Setup();
    Step_setup();
}

// ============================================================
// ENCENDER MOTOR
// ============================================================

void Stepper_ON() {

    if (!esp_timer_is_active(step_timer)) {

        uint64_t periodo_timer =
            (uint64_t)(60.0f * 1000000.0f /
            (STEP_RPMS * PASOS_X_VUELTA));

        ESP_ERROR_CHECK(
            esp_timer_start_periodic(step_timer, periodo_timer)
        );

        step_counter = 0;
    }
}

// ============================================================
// APAGAR MOTOR
// ============================================================

void Stepper_OFF() {

    if (esp_timer_is_active(step_timer)) {
        esp_timer_stop(step_timer);
    }

    GPIO.out_w1tc =
        (1UL << ST1) |
        (1UL << ST2) |
        (1UL << ST3) |
        (1UL << ST4);
}

bool Stepper_isRunning() {
    return esp_timer_is_active(step_timer);
}

// ============================================================
// MODULO: Anti_Atasco
// ============================================================
// ============================================================
// CONFIGURACIÓN DE PINES
// ============================================================

#define M_CRIBA       22

#define ACS712_PIN    35
#define RELE_CW_PIN   32
#define RELE_CCW_PIN  33

// ============================================================
// CONFIGURACIÓN DE RELÉS
// ============================================================

#define RELE_ACTIVO    0
#define RELE_INACTIVO  1

// ============================================================
// CONFIGURACIÓN DE CONTROL
// ============================================================

#define LIMITE_ATASCO_AMPERIOS  4.5f

#define SENSIBILIDAD_ACS        0.100f

#define DUTY_CRIBA              50

#define FREQ_PWM_CRIBA          20000
#define RES_PWM_CRIBA           10

// Ventana de adquisición
#define TM_MUESTREO             1000
#define TM_VENTANA              50000

// Tiempos de secuencia
#define TM_ESPERA               2000000
#define TM_REVERSA              2500000

// ============================================================
// VARIABLES INTERNAS
// ============================================================

static esp_timer_handle_t corriente_timer = NULL;
static esp_timer_handle_t secuencia_timer = NULL;

static bool secuencia_activa = false;

static uint8_t paso_secuencia = 0;

// Variables de adquisición ADC
static int valor_max = 0;
static int valor_min = 4095;

static int64_t tiempo_inicio = 0;

// ============================================================
// CONTROL DE SALIDAS
// ============================================================

void activarMotores() {

    // Asegurar que la reversa esté apagada
    GPIO.out_w1ts = (1UL << RELE_CCW_PIN);

    // Activar giro normal
    GPIO.out_w1tc = (1UL << RELE_CW_PIN);

    // Activar criba al duty definido
    uint32_t duty =
        (DUTY_CRIBA * ((1U << RES_PWM_CRIBA) - 1)) / 100;

    ledcWrite(M_CRIBA, duty);
}

// ------------------------------------------------------------

void apagarMotores() {

    // Apagar ambos relés
    GPIO.out_w1ts =
        (1UL << RELE_CW_PIN) |
        (1UL << RELE_CCW_PIN);

    // Apagar criba
    ledcWrite(M_CRIBA, 0);
}

// ============================================================
// CONTROL DE LA SECUENCIA ANTI-ATASCO
// ============================================================

static void Secuencia_callback(void *arg) {

    switch (paso_secuencia) {

        // ----------------------------------------------------
        // PASO 0: ESPERA CON MOTORES APAGADOS
        // ----------------------------------------------------

        case 0:

            Serial.println("Anti-atasco: Activando reversa");

            // Activar giro contrario
            GPIO.out_w1tc = (1UL << RELE_CCW_PIN);

            paso_secuencia = 1;

            ESP_ERROR_CHECK(
                esp_timer_start_once(secuencia_timer, TM_REVERSA)
            );

            break;

        // ----------------------------------------------------
        // PASO 1: FINALIZAR REVERSA
        // ----------------------------------------------------

        case 1:

            Serial.println("Anti-atasco: Finalizando reversa");

            // Apagar reversa
            GPIO.out_w1ts = (1UL << RELE_CCW_PIN);

            paso_secuencia = 2;

            ESP_ERROR_CHECK(
                esp_timer_start_once(secuencia_timer, TM_ESPERA)
            );

            break;

        // ----------------------------------------------------
        // PASO 2: FINALIZAR SECUENCIA
        // ----------------------------------------------------

        case 2:

            Serial.println("Anti-atasco: Retomando funcionamiento");

            paso_secuencia = 0;

            secuencia_activa = false;

            activarMotores();

            break;
    }
}

// ============================================================
// INICIO DE SECUENCIA
// ============================================================

static void iniciarSecuencia() {

    if (secuencia_activa) {
        return;
    }

    Serial.println("ATASCO DETECTADO");

    secuencia_activa = true;

    paso_secuencia = 0;

    // Apagar CW, CCW y criba
    apagarMotores();

    // Esperar 2 segundos antes de invertir
    ESP_ERROR_CHECK(
        esp_timer_start_once(secuencia_timer, TM_ESPERA)
    );
}

// ============================================================
// CÁLCULO DE CORRIENTE
// ============================================================

static float calcularCorriente() {

    float voltaje_ptp_adc =
        (valor_max - valor_min) * (3.3f / 4095.0f);

    // Compensación del divisor resistivo 10k / 22k
    float voltaje_ptp_sensor =
        voltaje_ptp_adc * (32.0f / 22.0f);

    // Conversión de pico a pico a RMS para señal sinusoidal
    float voltaje_rms =
        voltaje_ptp_sensor * 0.3535f;

    float amperios_rms =
        voltaje_rms / SENSIBILIDAD_ACS;

    return amperios_rms;
}

// ============================================================
// LECTURA PERIÓDICA DEL ACS712
// ============================================================

static void Corriente_callback(void *arg) {

    int lectura_adc = analogRead(ACS712_PIN);

    if (lectura_adc > valor_max) {
        valor_max = lectura_adc;
    }

    if (lectura_adc < valor_min) {
        valor_min = lectura_adc;
    }

    int64_t tiempo_actual = esp_timer_get_time();

    // Comprobar si terminó la ventana de adquisición
    if ((tiempo_actual - tiempo_inicio) >= TM_VENTANA) {

        float corriente_actual = calcularCorriente();

        /*Serial.print("Corriente RMS: ");
        Serial.print(corriente_actual);
        Serial.println(" A");*/

        // Reiniciar adquisición para la siguiente ventana
        valor_max = 0;
        valor_min = 4095;

        tiempo_inicio = tiempo_actual;

        // Detectar atasco únicamente si no hay secuencia activa
        if (!secuencia_activa &&
            corriente_actual > LIMITE_ATASCO_AMPERIOS) {

            iniciarSecuencia();
        }
    }
}

// ============================================================
// CONFIGURACIÓN GPIO
// ============================================================

static void GPIO_Setup() {

    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask =
        (1ULL << RELE_CW_PIN) |
        (1ULL << RELE_CCW_PIN);

    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Estado inicial: relés apagados
    GPIO.out_w1ts =
        (1UL << RELE_CW_PIN) |
        (1UL << RELE_CCW_PIN);
}

// ============================================================
// CONFIGURACIÓN PWM DE LA CRIBA
// ============================================================

static void Criba_Setup() {

    ledcAttach(
        M_CRIBA,
        FREQ_PWM_CRIBA,
        RES_PWM_CRIBA
    );

    ledcWrite(M_CRIBA, 0);
}

// ============================================================
// CONFIGURACIÓN DE TEMPORIZADORES
// ============================================================

static void Timers_Setup() {

    // Temporizador periódico de lectura
    const esp_timer_create_args_t corriente_args = {

        .callback = &Corriente_callback,

        .arg = NULL,

        .dispatch_method = ESP_TIMER_TASK,

        .name = "corriente_timer"
    };

    ESP_ERROR_CHECK(
        esp_timer_create(&corriente_args, &corriente_timer)
    );

    // Temporizador de secuencia one-shot
    const esp_timer_create_args_t secuencia_args = {

        .callback = &Secuencia_callback,

        .arg = NULL,

        .dispatch_method = ESP_TIMER_TASK,

        .name = "secuencia_timer"
    };

    ESP_ERROR_CHECK(
        esp_timer_create(&secuencia_args, &secuencia_timer)
    );
}

// ============================================================
// INICIALIZACIÓN DEL MÓDULO
// ============================================================

void Anti_Atasco_Setup() {

    analogReadResolution(12);

    GPIO_Setup();

    Criba_Setup();

    Timers_Setup();

    valor_max = 0;
    valor_min = 4095;

    tiempo_inicio = esp_timer_get_time();

    // Activar funcionamiento normal
    //activarMotores();

    // Iniciar lectura periódica del ACS712
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(
            corriente_timer,
            TM_MUESTREO
        )
    );

    Serial.println("Control anti-atasco iniciado");
}

bool Anti_Atasco_isRecovering() {
    return secuencia_activa;
}

// ============================================================
// MODULO: HMI_WiFi
// ============================================================
// ============================================================
// CONFIGURACION INTERNA
// ============================================================

#define WIFI_TASK_STACK        8192
#define WIFI_TASK_PRIORITY     1
#define WIFI_TASK_CORE         0

#define INTERVALO_ESTADO       500UL     // ms entre mensajes "state" (latido)
#define INTERVALO_RECONEXION   5000UL    // ms entre intentos de WiFi

// ============================================================
// VARIABLES
// ============================================================

struct MsgComando {
    HMIComando cmd;
    uint8_t    cliente;
};

struct MsgAck {
    uint8_t cliente;
    char    cmd[12];
    char    reason[32];
    bool    ok;
};

static WebSocketsServer webSocket(HMI_WS_PORT);

static bool servidorIniciado = false;

static QueueHandle_t colaComandos = NULL;   // Core 0 -> Core 1
static QueueHandle_t colaAck      = NULL;   // Core 1 -> Core 0

static HMIDatos datosHMI = {};
static portMUX_TYPE datosMux = portMUX_INITIALIZER_UNLOCKED;

static unsigned long tiempoEstado     = 0;
static unsigned long tiempoReconexion = 0;

// ============================================================
// UTILIDADES (Core 0)
// ============================================================

// Redondea a 1 decimal para un JSON corto. NaN se mantiene (sale "null").
static float r1(float v) {
    return isfinite(v) ? roundf(v * 10.0f) / 10.0f : v;
}

static void enviarAck(const MsgAck &m) {

    if (!servidorIniciado) return;

    JsonDocument doc;
    doc["type"]   = "ack";
    doc["cmd"]    = m.cmd;
    doc["ok"]     = m.ok ? 1 : 0;
    doc["reason"] = m.reason;

    String out;
    serializeJson(doc, out);

    webSocket.sendTXT(m.cliente, out);
}

static void ackDirecto(uint8_t cliente, const char *cmd, bool ok, const char *reason) {

    MsgAck m = {};
    m.cliente = cliente;
    m.ok = ok;
    strncpy(m.cmd, cmd, sizeof(m.cmd) - 1);
    strncpy(m.reason, reason, sizeof(m.reason) - 1);

    enviarAck(m);
}

// ============================================================
// RECIBIR COMANDOS (Core 0)
// ============================================================

static void procesarMensaje(uint8_t num, uint8_t *payload, size_t length) {

    JsonDocument doc;

    if (deserializeJson(doc, payload, length)) {
        ackDirecto(num, "unknown", false, "json_invalido");
        return;
    }

    const char *cmd = doc["cmd"] | "";

    MsgComando msg = {};
    msg.cliente = num;

    if      (strcmp(cmd, "start") == 0) msg.cmd = HMI_START;
    else if (strcmp(cmd, "stop")  == 0) msg.cmd = HMI_STOP;
    else if (strcmp(cmd, "bobinar") == 0) msg.cmd = HMI_BOBINAR;
    else {
        ackDirecto(num, "unknown", false, "comando_desconocido");
        return;
    }

    if (xQueueSend(colaComandos, &msg, 0) != pdPASS) {
        ackDirecto(num, cmd, false, "cola_llena");
    }
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {

    switch (type) {

        case WStype_CONNECTED:
            Serial.printf("HMI conectada (cliente %u)\n", num);
            break;

        case WStype_DISCONNECTED:
            Serial.printf("HMI desconectada (cliente %u)\n", num);
            break;

        case WStype_TEXT:
            procesarMensaje(num, payload, length);
            break;

        default:
            break;
    }
}

// ============================================================
// ENVIAR ESTADO (Core 0) - es tambien el "latido" de la HMI
// ============================================================

static void enviarEstado() {

    if (!servidorIniciado) return;
    if (webSocket.connectedClients() == 0) return;

    HMIDatos d;

    portENTER_CRITICAL(&datosMux);
    d = datosHMI;
    portEXIT_CRITICAL(&datosMux);

    JsonDocument doc;

    doc["type"] = "state";

    JsonArray micas = doc["micas"].to<JsonArray>();
    micas.add(r1(d.mica1));
    micas.add(r1(d.mica2));

    doc["rpm_ext"]    = r1(d.rpm_ext);
    doc["rpm_puller"] = r1(d.rpm_puller);
    doc["dia"]        = isfinite(d.diametro) ? roundf(d.diametro * 100.0f) / 100.0f : d.diametro;
    doc["spool_m"]    = r1(d.spool_m);

    doc["running"] = d.running ? 1 : 0;
    doc["estop"]   = d.estop   ? 1 : 0;

    JsonObject st = doc["stages"].to<JsonObject>();
    st["tritu"] = d.tritu ? 1 : 0;
    st["extru"] = d.extru ? 1 : 0;
    st["enfri"] = d.enfri ? 1 : 0;
    st["embo"]  = d.embo  ? 1 : 0;

    String out;
    serializeJson(doc, out);

    webSocket.broadcastTXT(out);
}

// ============================================================
// WIFI + SERVIDOR (Core 0)
// ============================================================

static void conectarWiFi() {

    Serial.println("Conectando a WiFi...");

    WiFi.begin(HMI_WIFI_SSID, HMI_WIFI_PASSWORD);

    tiempoReconexion = millis();
}

static void iniciarServidor() {

    if (MDNS.begin(HMI_MDNS_NAME)) {
        MDNS.addService("ws", "tcp", HMI_WS_PORT);
    } else {
        Serial.println("No se pudo iniciar mDNS");
    }

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocket.enableHeartbeat(15000, 3000, 2);

    servidorIniciado = true;

    Serial.println("\nServidor WebSocket para la HMI iniciado");
    Serial.print("IP ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.printf("HMI:  ws://%s.local:%d   (o ws://%s:%d)\n",
                  HMI_MDNS_NAME, HMI_WS_PORT,
                  WiFi.localIP().toString().c_str(), HMI_WS_PORT);
}

static void wifiLoop() {

    if (WiFi.status() != WL_CONNECTED) {

        if (millis() - tiempoReconexion >= INTERVALO_RECONEXION) {
            conectarWiFi();
        }

    } else if (!servidorIniciado) {

        iniciarServidor();
    }

    if (servidorIniciado) {

        webSocket.loop();

        // Respuestas (ACK) generadas por el Core 1
        MsgAck ack;
        while (xQueueReceive(colaAck, &ack, 0) == pdPASS) {
            enviarAck(ack);
        }
    }

    if (millis() - tiempoEstado >= INTERVALO_ESTADO) {
        tiempoEstado = millis();
        enviarEstado();
    }
}

static void wifiTask(void *parameter) {

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    conectarWiFi();

    for (;;) {
        wifiLoop();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ============================================================
// API PUBLICA
// ============================================================

void HMI_Setup() {

    colaComandos = xQueueCreate(8, sizeof(MsgComando));
    colaAck      = xQueueCreate(8, sizeof(MsgAck));

    if (colaComandos == NULL || colaAck == NULL) {
        Serial.println("ERROR HMI: no se pudieron crear las colas");
        return;
    }

    if (xTaskCreatePinnedToCore(
            wifiTask, "HMI_WiFi", WIFI_TASK_STACK, NULL,
            WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE) != pdPASS) {

        Serial.println("ERROR HMI: no se pudo crear la tarea WiFi");
    }
}

void HMI_publicarDatos(const HMIDatos &datos) {

    portENTER_CRITICAL(&datosMux);
    datosHMI = datos;
    portEXIT_CRITICAL(&datosMux);
}

bool HMI_leerComando(HMIComando &cmd, uint8_t &cliente) {

    if (colaComandos == NULL) return false;

    MsgComando msg;

    if (xQueueReceive(colaComandos, &msg, 0) != pdPASS) return false;

    cmd = msg.cmd;
    cliente = msg.cliente;
    return true;
}

void HMI_responder(uint8_t cliente, const char *cmd, bool ok, const char *reason) {

    if (colaAck == NULL) return;

    MsgAck m = {};
    m.cliente = cliente;
    m.ok = ok;
    strncpy(m.cmd, cmd, sizeof(m.cmd) - 1);
    strncpy(m.reason, reason, sizeof(m.reason) - 1);

    xQueueSend(colaAck, &m, 0);
}

// ============================================================
// PROGRAMA PRINCIPAL
// ============================================================
// ============================================================
// MODULOS
// ============================================================


// ============================================================
// CONFIGURACION DEL PROCESO
// ============================================================

// Temperaturas
#define SP_ZONA1 230.0f
#define SP_ZONA2 210.0f

#define TOLERANCIA_TEMP 2.0f

// Tiempo de estabilidad: 5 minutos
#define TIEMPO_ESTABILIDAD 300000UL

// Extrusora
#define SP_EXTRUSOR 40.0f

// Deteccion de filamento
#define UMBRAL_DETECCION_FILAMENTO 1.0f

// Monitor Serial
#define SERIAL_BAUD 115200

// HMI (pantalla web)
#define INTERVALO_HMI 100          // ms entre lecturas publicadas a la HMI

// Diametro del rodillo del puller en mm, para estimar los metros de
// filamento enrollados (spool_m). 0 = no calcular (la HMI muestra 0 m).
#define PULLER_DIAM_MM 0.0f

// ============================================================
// ESTADOS DE LA MAQUINA
// ============================================================

enum EstadoMaquina {

    ESPERA,
    CALENTANDO,
    ESTABILIZANDO,
    EXTRUYENDO,
    DETECTANDO_FILAMENTO,
    PRODUCIENDO_FILAMENTO,
    DETENIDA
};

EstadoMaquina estadoActual = ESPERA;

// ============================================================
// VARIABLES DE CONTROL
// ============================================================

bool antiAtascoInicializado = false;
bool diametroTimerInicializado = false;

bool bobinando = false;

unsigned long tiempoEstabilidad = 0;

unsigned long tiempoMonitor = 0;

const unsigned long INTERVALO_MONITOR = 1000;

unsigned long tiempoHMI = 0;
float spoolMetros = 0.0f;

// Buffer Serial
char comandoBuffer[32];
uint8_t indiceComando = 0;

// ============================================================
// FUNCIONES AUXILIARES
// ============================================================

bool temperaturaEstable() {

    float t1 = SSR1.tempEMA;
    float t2 = SSR2.tempEMA;

    if (!isfinite(t1) || !isfinite(t2)) {
        return false;
    }

    bool zona1 =
        (t1 >= SP_ZONA1 - TOLERANCIA_TEMP) &&
        (t1 <= SP_ZONA1 + TOLERANCIA_TEMP);

    bool zona2 =
        (t2 >= SP_ZONA2 - TOLERANCIA_TEMP) &&
        (t2 <= SP_ZONA2 + TOLERANCIA_TEMP);

    return zona1 && zona2;
}

// ------------------------------------------------------------

void imprimirEstado() {

    Serial.println("\n========== RECICLADORA ==========");

    Serial.print("Estado: ");

    switch (estadoActual) {

        case ESPERA:
            Serial.println("ESPERA");
            break;

        case CALENTANDO:
            Serial.println("CALENTANDO");
            break;

        case ESTABILIZANDO:
            Serial.println("ESTABILIZANDO");
            break;

        case EXTRUYENDO:
            Serial.println("EXTRUYENDO");
            break;

        case DETECTANDO_FILAMENTO:
            Serial.println("DETECTANDO FILAMENTO");
            break;

        case PRODUCIENDO_FILAMENTO:
            Serial.println("PRODUCIENDO FILAMENTO");
            break;

        case DETENIDA:
            Serial.println("DETENIDA");
            break;
    }

    Serial.println("--------------------------------");

    Serial.printf(
        "Temperatura Z1: %.2f C\n",
        SSR1.tempEMA
    );

    Serial.printf(
        "Temperatura Z2: %.2f C\n",
        SSR2.tempEMA
    );

    Serial.printf(
        "RPM Extrusora: %.2f\n",
        getRPM_Extrusor()
    );

    Serial.printf(
        "RPM Puller: %.2f\n",
        getRPM_Puller()
    );

    Serial.printf(
        "Diametro: %.2f\n",
        getDiameter()
    );

    Serial.printf(
        "Enfriamiento: %s\n",
        getCoolingState() ? "ACTIVO" : "INACTIVO"
    );

    Serial.printf(
        "Embobinado: %s\n",
        bobinando ? "ACTIVO" : "INACTIVO"
    );

    Serial.println("================================\n");
}

// ============================================================
// INICIALIZACION DE EXTRUSION
// ============================================================

void iniciarExtrusion() {

    Serial.println("\nIniciando extrusora...");

    // Setpoint de extrusora
    setSP_Extrusor(SP_EXTRUSOR);

    // Inicializar anti-atasco una sola vez.
    // Su Setup activa automaticamente la cribadora.

    if (!antiAtascoInicializado) {

        Anti_Atasco_Setup();

        antiAtascoInicializado = true;

    } else {

        activarMotores();
    }

    // Iniciar lectura del sensor de diametro
    if (!diametroTimerInicializado) {

        Diam_timer_Setup();

        diametroTimerInicializado = true;
    }

    estadoActual = EXTRUYENDO;

    Serial.println("Extrusion y cribadora activadas.");
}

// ============================================================
// CONTROL DE EMBONINADO
// ============================================================

// Devuelve true si el embobinado quedo activo; si no, "motivo" dice por que
bool iniciarBobinado(const char **motivo) {

    if (bobinando) {
        Serial.println("El embobinador ya esta activo.");
        if (motivo) *motivo = "ya_embobinando";
        return false;
    }

    if (estadoActual != PRODUCIENDO_FILAMENTO) {

        Serial.println(
            "No se puede bobinar: no hay filamento detectado."
        );

        if (motivo) *motivo = "sin_filamento";
        return false;
    }

    if (getDiameter() <= UMBRAL_DETECCION_FILAMENTO) {

        Serial.println("Lectura de diametro insuficiente.");
        if (motivo) *motivo = "diametro_insuficiente";
        return false;
    }

    setEmbobinando(true);

    Stepper_ON();

    bobinando = true;

    Serial.println("EMBOBINADO ACTIVADO");

    if (motivo) *motivo = "ok";
    return true;
}

// ------------------------------------------------------------

void detenerBobinado() {

    setEmbobinando(false);

    Stepper_OFF();

    bobinando = false;

    Serial.println("EMBOBINADO DETENIDO");
}

// ============================================================
// START
// ============================================================

void iniciarProceso() {

    if (estadoActual != ESPERA &&
        estadoActual != DETENIDA) {

        Serial.println("La maquina ya esta trabajando.");
        return;
    }

    Serial.println("\n================================");
    Serial.println("INICIANDO RECICLADORA");
    Serial.println("================================");

    detenerBobinado();

    // Setpoints termicos
    setSP(SSR1, SP_ZONA1);
    setSP(SSR2, SP_ZONA2);

    // Asegurar motores de proceso detenidos
    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    if (antiAtascoInicializado) {
        apagarMotores();
    }

    tiempoEstabilidad = 0;

    estadoActual = CALENTANDO;

    Serial.println("Calentamiento iniciado.");
    Serial.println("Zona 1: 230 C");
    Serial.println("Zona 2: 210 C");
}

// ============================================================
// STOP
// ============================================================

void detenerProceso() {

    Serial.println("\n!!! STOP !!!");

    // Detener embobinado
    detenerBobinado();

    // Detener extrusora y puller
    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    // Detener cribadora
    if (antiAtascoInicializado) {
        apagarMotores();
    }

    // Apagar calentamiento
    setSP(SSR1, 0.0f);
    setSP(SSR2, 0.0f);

    tiempoEstabilidad = 0;

    estadoActual = DETENIDA;

    Serial.println("Setpoints en cero.");
    Serial.println("Proceso detenido.");
}

// ============================================================
// MAQUINA DE ESTADOS
// ============================================================

void actualizarProceso() {

    switch (estadoActual) {

        // ----------------------------------------------------
        // ESPERA
        // ----------------------------------------------------

        case ESPERA:
        case DETENIDA:
            break;

        // ----------------------------------------------------
        // CALENTAMIENTO
        // ----------------------------------------------------

        case CALENTANDO:

            if (temperaturaEstable()) {

                tiempoEstabilidad = millis();

                estadoActual = ESTABILIZANDO;

                Serial.println(
                    "Temperaturas alcanzadas. "
                    "Iniciando estabilidad de 5 minutos."
                );
            }

            break;

        // ----------------------------------------------------
        // ESTABILIZACION
        // ----------------------------------------------------

        case ESTABILIZANDO:

            if (!temperaturaEstable()) {

                tiempoEstabilidad = 0;

                estadoActual = CALENTANDO;

                Serial.println(
                    "Temperatura fuera de rango. "
                    "Reiniciando estabilidad."
                );

                break;
            }

            if (millis() - tiempoEstabilidad >=
                TIEMPO_ESTABILIDAD) {

                Serial.println(
                    "Temperaturas estables durante 5 minutos."
                );

                iniciarExtrusion();
            }

            break;

        // ----------------------------------------------------
        // EXTRUSION
        // ----------------------------------------------------

        case EXTRUYENDO:

            estadoActual = DETECTANDO_FILAMENTO;

            Serial.println("Esperando deteccion de filamento.");

            break;

        // ----------------------------------------------------
        // DETECCION DE FILAMENTO
        // ----------------------------------------------------

        case DETECTANDO_FILAMENTO:

            if (getDiameter() >
                UMBRAL_DETECCION_FILAMENTO) {

                estadoActual = PRODUCIENDO_FILAMENTO;

                Serial.println(
                    "Filamento detectado. Produccion activa."
                );
            }

            break;

        // ----------------------------------------------------
        // PRODUCCION
        // ----------------------------------------------------

        case PRODUCIENDO_FILAMENTO:

            if (getDiameter() <=
                UMBRAL_DETECCION_FILAMENTO) {

                estadoActual = DETECTANDO_FILAMENTO;

                Serial.println(
                    "Filamento no detectado. "
                    "Esperando lectura."
                );
            }

            break;
    }
}

// ============================================================
// LECTURA SERIAL
// ============================================================

void procesarComando(char *comando) {

    // Convertir a mayusculas
    for (uint8_t i = 0; comando[i] != '\0'; i++) {

        comando[i] = toupper(comando[i]);
    }

    if (strcmp(comando, "START") == 0) {

        iniciarProceso();
    }

    else if (strcmp(comando, "STOP") == 0) {

        detenerProceso();
    }

    else if (strcmp(comando, "BOBINAR") == 0) {

        iniciarBobinado(nullptr);
    }

    else if (strcmp(comando, "DESBOBINAR") == 0) {

        detenerBobinado();
    }

    else if (strcmp(comando, "ESTADO") == 0) {

        imprimirEstado();
    }

    else {

        Serial.println("Comando no reconocido.");
    }
}

// ------------------------------------------------------------

void leerSerial() {

    while (Serial.available()) {

        char caracter = Serial.read();

        if (caracter == '\n' || caracter == '\r') {

            if (indiceComando > 0) {

                comandoBuffer[indiceComando] = '\0';

                procesarComando(comandoBuffer);

                indiceComando = 0;
            }
        }

        else if (indiceComando <
                 sizeof(comandoBuffer) - 1) {

            comandoBuffer[indiceComando++] = caracter;
        }
    }
}

// ============================================================
// HMI WIFI (se ejecuta en el Core 1; el WiFi corre en el Core 0)
// ============================================================

bool maquinaEnMarcha() {

    return estadoActual != ESPERA &&
           estadoActual != DETENIDA;
}

// ------------------------------------------------------------

void publicarDatosHMI() {

    unsigned long ahora = millis();

    // Metros enrollados: velocidad del puller x tiempo (solo si se
    // definio PULLER_DIAM_MM y el embobinado esta activo)
    static unsigned long ultimo = 0;
    float dt = (ahora - ultimo) / 1000.0f;
    ultimo = ahora;

    if (PULLER_DIAM_MM > 0.0f && bobinando) {

        spoolMetros +=
            getRPM_Puller() * 3.14159265f *
            (PULLER_DIAM_MM / 1000.0f) * dt / 60.0f;
    }

    HMIDatos d = {};

    d.mica1 = SSR1.tempEMA;
    d.mica2 = SSR2.tempEMA;

    d.rpm_ext    = getRPM_Extrusor();
    d.rpm_puller = getRPM_Puller();

    d.diametro = getDiameter();
    d.spool_m  = spoolMetros;

    d.running = maquinaEnMarcha();

    // No hay pulsador de emergencia leido por el codigo
    d.estop = false;

    // Etapas (solo indicador en la HMI)
    d.tritu = (estadoActual == EXTRUYENDO ||
               estadoActual == DETECTANDO_FILAMENTO ||
               estadoActual == PRODUCIENDO_FILAMENTO);
    d.extru = d.rpm_ext > 0.5f;
    d.enfri = getCoolingState();
    d.embo  = bobinando;

    HMI_publicarDatos(d);
}

// ------------------------------------------------------------

void procesarComandosHMI() {

    HMIComando cmd;
    uint8_t cliente;

    while (HMI_leerComando(cmd, cliente)) {

        if (cmd == HMI_START) {

            if (maquinaEnMarcha()) {

                HMI_responder(cliente, "start", false, "ya_en_marcha");

            } else {

                iniciarProceso();
                HMI_responder(cliente, "start", true, "ok");
            }
        }

        else if (cmd == HMI_BOBINAR) {

            const char *motivo = "ok";
            bool ok = iniciarBobinado(&motivo);
            HMI_responder(cliente, "bobinar", ok, motivo);
        }

        else if (cmd == HMI_STOP) {

            detenerProceso();
            HMI_responder(cliente, "stop", true, "ok");
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(SERIAL_BAUD);

    delay(500);

    Serial.println("\n================================");
    Serial.println("RECICLADORA DE FILAMENTO");
    Serial.println("CONTROL MAESTRO");
    Serial.println("================================");

    // Control termico
    thermalSetup();

    // Control de motores y PID
    Motor_control_Setup();

    // Post extrusion
    Post_Extrusion_Setup();

    // Stepper
    Stepper_Init();

    Stepper_OFF();

    // Setpoints iniciales
    setSP(SSR1, 0.0f);
    setSP(SSR2, 0.0f);

    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    setEmbobinando(false);

    estadoActual = ESPERA;

    // Comunicacion con la HMI (WiFi + WebSocket en el Core 0)
    HMI_Setup();

    Serial.println("\nSistema inicializado.");
    Serial.println("Esperando comando START.");

    Serial.println(
        "Comandos: START, STOP, BOBINAR, "
        "DESBOBINAR, ESTADO"
    );
}

// ============================================================
// LOOP
// ============================================================

void loop() {

    leerSerial();

    actualizarProceso();

    // Comandos Start / Stop recibidos de la HMI
    procesarComandosHMI();

    // Publicar variables para la HMI
    if (millis() - tiempoHMI >= INTERVALO_HMI) {

        tiempoHMI = millis();

        publicarDatosHMI();
    }

    // Monitor de variables cada segundo
    if (millis() - tiempoMonitor >= INTERVALO_MONITOR) {

        tiempoMonitor = millis();

        if (estadoActual != ESPERA &&
            estadoActual != DETENIDA) {

            imprimirEstado();
        }
    }
}
