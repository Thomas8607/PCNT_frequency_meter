#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"


#define SIGNAL_INPUT GPIO_NUM_4

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

typedef struct {
    uint32_t timeStamp; // The time the event occured
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
static void IRAM_ATTR pcnt_intr_handler(void *arg) {
    uint32_t currentMicros = micros(); //Time at instant ISR was called
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
/* Initialize PCNT functions for one channel:
 *  - configure and initialize PCNT with pos-edge counting 
 *  - set up the input filter
 *  - set up the counter events to watch
*/
void pcnt_init_channel(int PCNT_INPUT_SIG_IO) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config; 
        // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
        pcnt_config.ctrl_gpio_num = -1;
        pcnt_config.channel = PCNT_CHANNEL_0;
        pcnt_config.unit = PCNT_UNIT_0;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
        pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = 19;
        pcnt_config.counter_l_lim = -20;
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT_0);
    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
}

/* Count RPM Function - takes first timestamp and last timestamp,
number of pulses, and pulses per revolution */
int countRPM(uint32_t firstTime, uint32_t lastTime, uint16_t pulseTotal, uint16_t pulsePerRev) {
    uint32_t timeDelta = (lastTime - firstTime);                        // lastTime - firstTime
    if (timeDelta <= 0){                                                // This means we've gotten something wrong
        return -1;
    }
    //return ((60000 * (pulseTotal/pulsePerRev)) / timeDelta);          // Fordulat/perc
    Serial.println("timeDelta: " + String(timeDelta));                  // 20658usec
    return ((1000000 * (pulseTotal/pulsePerRev)) / timeDelta);          // Frekvencia 968Hz
}
void setup() {
    Serial.begin(115200);
}

void loop() {
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_init_channel(SIGNAL_INPUT); // Initialize Unit 0 to pin 4
    int16_t RPM = -1; // Fan 0 RPM
    uint32_t lastStamp = 0; //for previous time stamp for fan 0
    pcnt_evt_t evt;
    portBASE_TYPE res;
    for(;;){
        /* Wait for the event information passed from PCNT's interrupt handler.
        * Once received, decode the event type and print it on the serial monitor.
        */
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            if (lastStamp == 0) {
                lastStamp = evt.timeStamp;
            }
            RPM = countRPM(lastStamp, evt.timeStamp, 20, 1);
            if (RPM == -1) {
                Serial.println("RPM Calc error detected!");
                continue;
            }
            lastStamp = evt.timeStamp;
        }
        else {
            RPM = 0;
        }
        Serial.println("Frekvencia: " + String(RPM));
    }
    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }
}