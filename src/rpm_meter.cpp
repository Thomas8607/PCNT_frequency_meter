#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

#define SIGNAL_INPUT GPIO_NUM_4                        // Pulse counter input

TaskHandle_t RpmCalculationHandle = NULL;
xQueueHandle pcnt_evt_queue;                           // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL;              // user's ISR service handle
int16_t PulseperRev, PulseTotal, RPM;
typedef struct {
    uint32_t timeStamp;                                // The time the event occured
} pcnt_evt_t;

// PCNT interrupt
static void IRAM_ATTR pcnt_intr_handler(void *arg) {
    uint32_t currentMicros = micros();                 // Time at instant ISR was called
    uint32_t intr_status = PCNT.int_st.val;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    if (intr_status & (BIT(0))) {
        evt.timeStamp = currentMicros; 
        PCNT.int_clr.val = BIT(0);
        xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
        if (HPTaskAwoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// Initialize PCNT functions for one channel
void pcnt_init(int16_t PCNT_INPUT_SIG_IO, int16_t PCNT_counter_h_limit) {
    pcnt_config_t pcnt_config; 
        pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
        pcnt_config.ctrl_gpio_num = -1;
        pcnt_config.channel = PCNT_CHANNEL_0;
        pcnt_config.unit = PCNT_UNIT_0;
        pcnt_config.pos_mode = PCNT_COUNT_DIS;
        pcnt_config.neg_mode = PCNT_COUNT_INC;
        pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
        pcnt_config.counter_h_lim = 19;
        pcnt_config.counter_l_lim = -1 * PCNT_counter_h_limit;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

// Count RPM Function
int16_t countRPM(uint32_t firstTime, uint32_t lastTime, uint16_t pulsePerRev, int16_t pulseTotal) {
    uint32_t timeDelta = (lastTime - firstTime);
    if (timeDelta == 0) {
       return 0;
    }
    return ((1000000 * ((float)pulseTotal/(float)pulsePerRev)) / timeDelta);            // Frequency
    // return ((60 * 1000000 * ((float)pulseTotal/(float)pulsePerRev)) / timeDelta);    // Rpm
}
// Rpm calculation Task
void RpmCalculationTask(void *arg) {
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_init(SIGNAL_INPUT, PulseTotal);   // Initialize PCNT
    uint32_t lastStamp = 0;                // for previous timestamp
    pcnt_evt_t evt;
    portBASE_TYPE res;
    while(1) {
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            if (lastStamp == 0) {
                lastStamp = evt.timeStamp;
            }
            RPM = countRPM(lastStamp, evt.timeStamp, PulseperRev, PulseTotal);
            lastStamp = evt.timeStamp;
        }
        else {
            RPM = 0;
        }
    }
    // Free the ISR service handle
    if(user_isr_handle) {                                                      
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }
}


void setup() {
    Serial.begin(115200);
    xTaskCreate(RpmCalculationTask, "RpmCalculationTask", 4096, NULL, 10, &RpmCalculationHandle);
}

void loop() {
    PulseperRev = 1;
    PulseTotal = 10;
    Serial.println("Frequency: " + String(RPM));
    //Serial.println("Rpm: " + String(RPM));
}




