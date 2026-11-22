#include <Arduino.h>
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_timer.h"

#define PWM_TEST 2

#define TIMEOUT_PULSOS_US 500000  // 500 ms

#define FREQ_Puller   35
#define FREQ_Extrusor   39

#define PWM_Puller    23
#define PWM_Extrusor  14

#define PWM_FREQ      70000
#define PWM_RESOL     10

#define EMA_ALPHA 0.2f
#define EMA_CV_ALPHA 0.2f

#define PULSES_PER_REV_Puller  11.0f
#define GEAR_RATIO_Puller      21.3f

// --- PID ---
float sp_puller = 0; // Setpoint
float pv_puller = 0; // Variable de proceso (frecuencia medida por el MCPWM)

float cv_puller = 0;
float cv1_puller = 0;
float error_puller = 0;
float error1_puller = 0;
float error2_puller = 0;

float Kp_puller = 1.4410;
float Ki_puller = 4.5246;
float Kd_puller = 0.000;

// --- PID EXTRUSOR ---
float sp_extrusor = 0; // Setpoint
float pv_extrusor = 0; // Variable de proceso (frecuencia medida por el MCPWM)

float cv_extrusor = 0;
float cv1_extrusor = 0;
float error_extrusor = 0;
float error1_extrusor = 0;
float error2_extrusor = 0;

float Kp_extrusor = 2.5705;
float Ki_extrusor = 2.8041;
float Kd_extrusor = 0.000;

//--- Tiempo Muestreo Motores ---
float Tm = 0.05;


// --- Timer ---
const uint64_t CALC_PERIOD_US = 100000;   // 50 ms
const uint8_t  MISSED_LIMIT   = 20;      // 20 * 50 ms = 1 s sin flancos -> 0 Hz

typedef struct {
  uint32_t last_cap;          // último valor capturado
  volatile uint32_t period;   // ticks entre dos flancos de subida
  volatile bool     valid;    // hubo medición nueva
  bool              first;    // ya hay una referencia previa
  volatile uint32_t last_pulse_us;
} cap_data_t;

static cap_data_t data_a = {0, 0, false, false};
static cap_data_t data_b = {0, 0, false, false};

static uint32_t res_hz = 0;


// --- RPMS ---
volatile float rpm_puller    = 0.0f;
volatile float rpm_extrusor    = 0.0f;

// --- RPMS EMA ---
volatile float rpmEMA_puller     = 0.0f;
volatile float rpmEMA_extrusor   = 0.0f;
bool emaInitialized_puller = false;
bool emaInitialized_extrusor = false;

// --- RPMS CV ---

volatile float cvEMA_puller = 0.0f;
volatile float cvEMA_extrusor = 0.0f;
bool cvEmaInitialized_puller = false;
bool cvEmaInitialized_extrusor = false;

// Se ejecuta en ISR: calcula el periodo entre flancos de subida consecutivos
static bool IRAM_ATTR cap_callback(mcpwm_cap_channel_handle_t chan,
                                   const mcpwm_capture_event_data_t *edata,
                                   void *user_data)
{
  cap_data_t *d = (cap_data_t *)user_data;
  d->last_pulse_us = (uint32_t)esp_timer_get_time();

  if (d->first) {
    // La resta en uint32_t maneja bien el desbordamiento del contador
    d->period = edata->cap_value - d->last_cap;
    d->valid = true;
  }
  d->last_cap = edata->cap_value;
  d->first = true;
  return false;
}

static void setup_channel(mcpwm_cap_timer_handle_t timer, int gpio, cap_data_t *data)
{
  mcpwm_cap_channel_handle_t chan = NULL;
  mcpwm_capture_channel_config_t cfg = {};
  cfg.gpio_num = gpio;
  cfg.prescale = 1;
  cfg.flags.pos_edge = true;     // captura en flanco de subida
  cfg.flags.neg_edge = false;
  cfg.flags.pull_up = false;     // GPIO35/39 no tienen pull-up interno
  cfg.flags.pull_down = false;
  ESP_ERROR_CHECK(mcpwm_new_capture_channel(timer, &cfg, &chan));

  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = cap_callback;
  ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(chan, &cbs, data));
  ESP_ERROR_CHECK(mcpwm_capture_channel_enable(chan));
}

void setPWM(uint8_t pin,uint8_t duty){
  int pwm = constrain(duty*1023.0f/100.0f, 0.0f, 1023.0f);
  ledcWrite(pin, pwm);
}

// Corre cada 50 ms en contexto de tarea (float permitido)

static void onTimer50ms(void *arg) {

  float freq_a = 0.0f, freq_b = 0.0f;
  uint32_t ahora_us = (uint32_t)esp_timer_get_time();

  if (data_a.valid && data_a.period > 0) {
    freq_a = (float)res_hz / (float)data_a.period;
    rpm_puller = freq_a * 60.0f / (PULSES_PER_REV_Puller * GEAR_RATIO_Puller);

    // Filtro EMA
    if (!emaInitialized_puller) {
      rpmEMA_puller = rpm_puller;
      emaInitialized_puller = true;
      cvEmaInitialized_puller = true;
    } 
    else {
      rpmEMA_puller = EMA_ALPHA * rpm_puller +
               (1.0f - EMA_ALPHA) * rpmEMA_puller;
    }
  }
  if (data_b.valid && data_b.period > 0) {
    freq_b = (float)res_hz / (float)data_b.period;
    rpm_extrusor = freq_b * 60.0f / (PULSES_PER_REV_Puller * GEAR_RATIO_Puller);
    // Filtro EMA
    if (!emaInitialized_extrusor) {
        rpmEMA_extrusor = rpm_extrusor;
        emaInitialized_extrusor = true;
        cvEmaInitialized_extrusor = true;
    }
    else {
        rpmEMA_extrusor = EMA_ALPHA * rpm_extrusor +
                          (1.0f - EMA_ALPHA) * rpmEMA_extrusor;
    }
  }

  // PULLER
  if ((uint32_t)(ahora_us - data_a.last_pulse_us) > TIMEOUT_PULSOS_US) {
      rpm_puller = 0;
      rpmEMA_puller = 0;
      //Serial.printf("ahora - antes: %d|| ",ahora_us - data_a.last_pulse_us );
  }

  // EXTRUSOR
  if ((uint32_t)(ahora_us - data_b.last_pulse_us) > TIMEOUT_PULSOS_US) {
      rpm_extrusor = 0;
      rpmEMA_extrusor = 0;
  }

  // Si no llegan flancos nuevos, la próxima lectura reportará 0 Hz
  data_a.valid = false;
  data_b.valid = false;
  Serial.printf("GPIO%d: %.2f RPM | GPIO%d: %.2f RPM\n",
                FREQ_Puller, rpmEMA_puller, FREQ_Extrusor, rpm_extrusor);


  // PID PULLER

  // ---- SET POINT Y ERROR ----
  error_puller = sp_puller - rpmEMA_puller;

  // ---- ECUACIÓN DE DIFERENCIAS ----
  cv_puller = cv1_puller
            + (Kp_puller + Kd_puller/Tm) * error_puller
            + (-Kp_puller + Ki_puller*Tm - 2*Kd_puller/Tm) * error1_puller
            + (Kd_puller/Tm) * error2_puller;

  cv_puller = constrain(cv_puller, 0.0f, 100.0f);

  // ---- FILTRO EMA DE LA SALIDA ----
  if (!cvEmaInitialized_puller) {
      cvEMA_puller = cv_puller;
      cvEmaInitialized_puller = true;
  }
  else {
      cvEMA_puller = EMA_CV_ALPHA * cv_puller +
                    (1.0f - EMA_CV_ALPHA) * cvEMA_puller;
  }

  // ---- ACTUALIZAR ESTADOS DEL CONTROLADOR ----
  cv1_puller = cv_puller;
  error2_puller = error1_puller;
  error1_puller = error_puller;

  // ---- APLICAR PWM FILTRADO ----
  setPWM(PWM_Puller,cvEMA_puller);


  // PID EXTRUSOR

  // ---- SET POINT Y ERROR ----
  error_extrusor = sp_extrusor - rpmEMA_extrusor;

  // ---- ECUACIÓN DE DIFERENCIAS ----
  cv_extrusor = cv1_extrusor
              + (Kp_extrusor + Kd_extrusor/Tm) * error_extrusor
              + (-Kp_extrusor + Ki_extrusor*Tm - 2*Kd_extrusor/Tm) * error1_extrusor
              + (Kd_extrusor/Tm) * error2_extrusor;

  cv_extrusor = constrain(cv_extrusor, 0.0f, 100.0f);

  // ---- FILTRO EMA DE LA SALIDA ----
  if (!cvEmaInitialized_extrusor) {
      cvEMA_extrusor = cv_extrusor;
      cvEmaInitialized_extrusor = true;
  }
  else {
      cvEMA_extrusor = EMA_CV_ALPHA * cv_extrusor +
                      (1.0f - EMA_CV_ALPHA) * cvEMA_extrusor;
  }

  // ---- ACTUALIZAR ESTADOS DEL CONTROLADOR ----
  cv1_extrusor = cv_extrusor;
  error2_extrusor = error1_extrusor;
  error1_extrusor = error_extrusor;

  // ---- APLICAR PWM FILTRADO ----
  setPWM(PWM_Extrusor, cvEMA_extrusor);
  return;
}

void setupCalcTimer() {
  esp_timer_handle_t calcTimer = NULL;
  esp_timer_create_args_t args = {};
  args.callback        = &onTimer50ms;
  args.name            = "rpm_calc";
  args.dispatch_method = ESP_TIMER_TASK;
  ESP_ERROR_CHECK(esp_timer_create(&args, &calcTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(calcTimer, CALC_PERIOD_US));
}
void setup()
{
  Serial.begin(115200);
  delay(500);

  // Un único timer de captura (grupo 0) compartido por ambos canales
  mcpwm_cap_timer_handle_t cap_timer = NULL;
  mcpwm_capture_timer_config_t timer_cfg = {};
  timer_cfg.group_id = 0;
  timer_cfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;   // APB = 80 MHz
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_cfg, &cap_timer));

  ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(cap_timer, &res_hz));
  Serial.printf("Resolucion del timer de captura: %lu Hz\n", (unsigned long)res_hz);

  setup_channel(cap_timer, FREQ_Puller, &data_a);
  setup_channel(cap_timer, FREQ_Extrusor, &data_b);

  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

  ledcAttach(PWM_Extrusor, PWM_FREQ, PWM_RESOL);
  ledcAttach(PWM_Puller, PWM_FREQ, PWM_RESOL);
  ledcAttach(PWM_TEST, PWM_FREQ, PWM_RESOL);

  setupCalcTimer();      // al final, cuando capResolutionHz ya está definido
}

void loop()
{
  delay(1000);
  /*setPWM(PWM_Extrusor, 50);
  setPWM(PWM_Puller, 50);
  setPWM(PWM_TEST, 50);*/

  if (Serial.available() > 0) {
    String datos = Serial.readStringUntil('\n');

    int separador = datos.indexOf(',');

    if (separador != -1) {
        sp_puller = datos.substring(0, separador).toFloat();
        sp_extrusor = datos.substring(separador + 1).toFloat();
    }
  }

  /*float freq_a = 0.0f, freq_b = 0.0f;

  if (data_a.valid && data_a.period > 0) {
    freq_a = (float)res_hz / (float)data_a.period;
  }
  if (data_b.valid && data_b.period > 0) {
    freq_b = (float)res_hz / (float)data_b.period;
  }

  // Si no llegan flancos nuevos, la próxima lectura reportará 0 Hz
  data_a.valid = false;
  data_b.valid = false;

  Serial.printf("GPIO%d: %.2f Hz | GPIO%d: %.2f Hz\n",
                FREQ_Puller, freq_a, FREQ_Extrusor, freq_b);*/
}