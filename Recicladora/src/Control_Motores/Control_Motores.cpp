
#include "Control_Motores.h"

#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_timer.h"

// ============================================================
// 1. CONFIGURACION
// ============================================================

#define TIMEOUT_PULSOS_US 500000
#define TIMER1_PERIOD     50000

#define SENSOR_DIAM 36

#define FREQ_Puller   35
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

static esp_timer_handle_t calcTimer = NULL;

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
        esp_timer_create(&args, &calcTimer)
    );

    ESP_ERROR_CHECK(
        esp_timer_start_periodic(calcTimer, CALC_PERIOD_US)
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
