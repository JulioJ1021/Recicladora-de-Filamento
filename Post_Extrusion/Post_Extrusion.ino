#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_timer.h"

#define DIAM_SENS 36
#define VENT      23
#define M_EMB     4

#define UMBRAL_DIAM 10

#define TM_DIAM 50000 //50 ms

#define FREQ_PWM_EMB     20000
#define RES_PWM_EMB      10        // Rango: 0 a 1023
#define DUTY_EMB         512       // ~50% de 12V -> ~6V efectivos de tope

bool ENFRIANDO = false;
float diameter = 0.0f, rpm_puller = 0.0f;
esp_timer_handle_t diameter_timer = NULL;

void Post_Extrusion_Setup(){
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << VENT);
  io_conf.mode         = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type    = GPIO_INTR_DISABLE;
  
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ledcAttach(M_EMB, FREQ_PWM_EMB, RES_PWM_EMB);
  ledcWrite(M_EMB, 0); 

  analogReadResolution(10);
}

float diam2rpms(float diam){
  float rpms = 2.0f*diam - 0; //temp function
  return rpms;
}

float diam_calc(int sensor_read){
  float diam = sensor_read/2.0f;
  return diam;
}

void Diam_manage(void *arg){
  diameter = diam_calc(analogRead(DIAM_SENS));
  ENFRIANDO = (diameter > UMBRAL_DIAM) ? true:false;
  if(ENFRIANDO){
    GPIO.out_w1ts = (1ul << VENT);
    ledcWrite(M_EMB, DUTY_EMB);
    Serial.println("Frio");
    rpm_puller = diam2rpms(diameter);
    //setpoint rpms acorde a rpm puller
    //considerar PID para el diámetro del filamento
  }else{
    GPIO.out_w1tc = (1ul << VENT);
    ledcWrite(M_EMB, 0);
    //setpoint rpms en 0

  }
}

void Diam_timer_Setup(){
  const esp_timer_create_args_t timer_args= {
    .callback = &Diam_manage,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,  
    .name = "Diameter sensor"         
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &diameter_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(diameter_timer, TM_DIAM));
}

void setup() {
  Serial.begin(115200);
  Post_Extrusion_Setup();
  Diam_timer_Setup();
}

void loop() {
  // put your main code here, to run repeatedly:

}
