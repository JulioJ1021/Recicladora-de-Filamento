
#include "Post_Extrusion.h"

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_timer.h"
#include "esp_err.h"

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
static bool EMBOBINANDO = false

static float diameter = 0.0f;
static float rpm_puller = 0.0f;

static esp_timer_handle_t diameter_timer = NULL;

// ============================================================
// FUNCIONES INTERNAS
// ============================================================

static float diam2rpms(float diam) {

    float rpms = 2.0f * diam - 0; // Función temporal

    return rpms;
}

static float diam_calc(int sensor_read) {

    float diam = sensor_read / 2.0f;

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

        rpm_puller = diam2rpms(diameter);

        // Setpoint RPM acorde a RPM del puller
        // Considerar PID para el diámetro del filamento

    } else {

        GPIO.out_w1tc = (1UL << VENT);

        ledcWrite(M_EMB, 0);

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
