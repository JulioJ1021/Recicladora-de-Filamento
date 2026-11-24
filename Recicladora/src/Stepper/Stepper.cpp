#include "Stepper.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_err.h"

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

    Serial.println("n");
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