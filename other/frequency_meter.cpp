#include "Arduino.h"
#include "driver/pcnt.h"      
#include "esp32-hal-timer.h" 
#include "esp_timer.h" 
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h" 
#include "soc/pcnt_struct.h"                             

#define PCNT_H_LIM_VAL        overflow                                  
#define FREQ_PIN              35

bool            flag          = true;                                   
uint32_t        overflow      = 20000;                                  
int16_t         pulses        = 0;                                  
uint32_t        overflow_cnt  = 0;                                        
volatile double frequency     = 0;
uint16_t result = 0;
static const char *TAG = "MyTimerModule";

void pcnt_get_counter(void *p); 
void pcnt_event_handler(void *arg);
void pcnt_init(void);

esp_timer_create_args_t timer_args;                                       // Create an esp_timer instance
esp_timer_handle_t timer_handle;                                          // Create an single timer
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

pcnt_config_t pcnt_config = {
  .pulse_gpio_num    = FREQ_PIN,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_INC,
  .counter_h_lim     = 20000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_0, 
  .channel           = PCNT_CHANNEL_0
  };

// Counting overflow pulses
void IRAM_ATTR pcnt_event_handler(void *arg) {
  portENTER_CRITICAL_ISR(&timer_mux);                                    // disable interrupt
  overflow_cnt++;                                                        // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_UNIT_0);                                   // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timer_mux);                                     // enable interrupt
}                                        
//----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                                  
  pcnt_init();                                                             // Initialize
}
//----------------------------------------------------------------------------------
void pcnt_init(void) {  
  pinMode(FREQ_PIN,INPUT);
  pcnt_unit_config(&pcnt_config);
  pcnt_isr_register(pcnt_event_handler, NULL, 0, NULL);                   // Setup Register ISR handler
  pcnt_intr_enable(PCNT_UNIT_0);  
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0); 
  pcnt_counter_pause(PCNT_UNIT_0);                                        // Pause PCNT unit
  pcnt_counter_clear(PCNT_UNIT_0);                                        // Clear PCNT unit 
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);                         // Enable event to watch - max count
  pcnt_counter_resume(PCNT_UNIT_0);                                       // Resume PCNT unit - starts count
  timer_args.callback = pcnt_get_counter;
  timer_args.arg      = NULL;
  timer_args.name     = "one shot timer";
  if(esp_timer_create(&timer_args, &timer_handle) != ESP_OK) {
    ESP_LOGE(TAG, "timer create");
  }
  timer_args.callback = pcnt_get_counter;                                 // Set esp-timer argument
  esp_timer_create(&timer_args, &timer_handle);                           // Create esp-timer instance
}
//----------------------------------------------------------------------------------
void pcnt_get_counter(void *p) {                                                  // Read Pulse Counter
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*) &result); 
  flag = true;
}
//---------------------------------------------------------------------------------
void loop() {
  if (flag == true){
    flag = false;
    frequency =  result + (overflow_cnt*20000); 
    overflow_cnt = 0; 
    pcnt_counter_clear(PCNT_UNIT_0); 
    pcnt_counter_resume(PCNT_UNIT_0); 
    overflow_cnt = 0;    
  
    Serial.println("Frekvencia: " + String(frequency/2) + "Hz");

    pcnt_counter_clear(PCNT_UNIT_0);
    esp_timer_start_once(timer_handle, 1000000);                    // Initialize High resolution timer (1 sec)
  }
    // Put your function here, if you want
}