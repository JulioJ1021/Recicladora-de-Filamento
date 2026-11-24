#include "Anti_Atasco.h"

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_timer.h"
#include "esp_err.h"

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