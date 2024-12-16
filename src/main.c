/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/********************** inclusions *******************************************/
/* Project includes. */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
//              DESACTIVAR EL WATCHDOG
// #define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 0
// #define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 0

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/dac.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "ADC_continuous.h"
#include <math.h>
#include "buffer.h"
#include "esp_task_wdt.h"


/********************** macros and definitions *******************************/
#define UPDATE_PERIOD_MICROSECONDS 50  //[T]=usec
#define UPDATE_SAMPLE_FRECUENCY 1000000.0/UPDATE_PERIOD_MICROSECONDS  //[Fs]= Hz
#define N  (int) ((UPDATE_SAMPLE_FRECUENCY)/DOn1)


#define DAC_CHANNEL DAC_CHAN_0
#define BUTTON_EFFECT_1 GPIO_NUM_16
#define BUTTON_EFFECT_2 GPIO_NUM_17
#define BUTTON_EFFECT_3 GPIO_NUM_18
#define BUTTON_EFFECT_4 GPIO_NUM_19
#define BUTTON_EFFECT_5 GPIO_NUM_22

// Echo defines
#define DELAY_BUFFER_SIZE 20000
#define FIXED_DECIMAL_ONE 1000
#define FIXED_DECIMAL(x) ( (int)((x) *  FIXED_DECIMAL_ONE) )
#define MIX(v1, v2, beta) ( ((v1) * beta + (FIXED_DECIMAL_ONE - beta) * (v2)) / FIXED_DECIMAL_ONE)
#define ECHO_FEEDBACK  FIXED_DECIMAL(0.5)
#define HISTORY_LENGTH (1<<14)
#define HISTORY_MASK (HISTORY_LENGTH - 1)

// Tremolo defines
#define TREMOLO_RATE 22000
#define TREMOLO_FREQUENCY 4
#define TREMOLO_INCREMENT 2 * M_PI * TREMOLO_FREQUENCY / TREMOLO_RATE

/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
	uint32_t i =0;
/********************** external data declaration *****************************/

enum EFFECTS{
    NO_EFFECT,
    TEST,
    ECHO,
    ECHO_SIMPLE, // OK
    REVERB,
    TREMOLO,
    BITCRUSHER,
    SATURATION,
};

/********************** external functions definition ************************/

// Echo
static long echo_buffer[DELAY_BUFFER_SIZE] = {0};
static long past_samples[DELAY_BUFFER_SIZE] = {0};
static int delay_index = 0;

// Tremolo
float tremolo_phase = 0.0;

CircularBuffer_t *circular_buffer = NULL;

void app_main(void)
{
    //GPIO Initialize
    gpio_config_t io_config;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.pin_bit_mask = (1ULL << GPIO_NUM_32)|(1ULL << GPIO_NUM_33);//|(1ULL << GPIO_NUM_35);
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    gpio_config(&io_config);

    //ADC initialize
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
//    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));


    //DAC Initialize
    dac_output_enable(DAC_CHANNEL);

    // Variables para mantener el tiempo de actualización
    uint64_t current_time = 0;
    int i =0;

    long data, data_out, old;
    while (1) 
    {
        gpio_set_level(GPIO_NUM_32, 1);
        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
        gpio_set_level(GPIO_NUM_32, 0);
        if (ret == ESP_OK) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
            data = EXAMPLE_ADC_GET_DATA(p);
            data -= 2047;
            // data -= 1900; // 1.77V

            data_out = data;

            switch (NO_EFFECT)
            {
            case TEST:
                
                break;

            case ECHO_SIMPLE:
                old = past_samples[delay_index];
                past_samples[delay_index] = data;
                delay_index = (delay_index + 1) % ((DELAY_BUFFER_SIZE * 100UL) / 1000UL);
                
                data_out = (data + 0.65*old) / 2;
                break;
            case NO_EFFECT:
                break;

            case REVERB:
                {
                    // Reverb
                old = past_samples[delay_index];
                past_samples[delay_index] = data;
                delay_index = (delay_index + 1) % (DELAY_BUFFER_SIZE - 1);
                
                data_out = (data + 0.5*old) / 2;
                }
                break;

            case ECHO:
                old = past_samples[delay_index];
                data_out = data + 0.5*old;
                past_samples[delay_index] = data_out;
                delay_index = (delay_index + 1) % ((DELAY_BUFFER_SIZE * 300UL) / 1000UL);
                break;

            case TREMOLO:
                float lfo_value = (sin(tremolo_phase) + 1.0) / 2.1 + 0.1;
                data_out *= lfo_value;
                tremolo_phase += TREMOLO_INCREMENT;
                if (tremolo_phase >= 2 * M_PI) {
                    tremolo_phase -= 2 * M_PI;
                }
                break;
            case SATURATION:
                // The two-time
                data_out *= 4;
                data_out = (data_out > 1000) ? (1000) : (data_out);
                data_out = (data_out < -1000) ? (-1000) : (data_out);
                break;
            case BITCRUSHER:
                // data_out
                data_out += 4096;
                data_out = (data_out >> 8) << 8;
                data_out -= 4096;
                break; 
            default:
                break;
            }


            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_33, 0);
            while ((esp_timer_get_time() - current_time) < (UPDATE_PERIOD_MICROSECONDS));
            
            
            // vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MICROSECONDS / 1000));
            // data_out += 1900;
            // data_out += 2180;
            data_out += 2047;
            // clip data out
            if (data_out > 4090) {
                data_out = 4090;
            } else if (data_out < 10) {
                data_out = 10;
            }
            // int scaled_value = data_out / 255 * 3;
            uint8_t pre_out = (uint8_t) (data_out >>4 & 0x000000FF);

            // Clip de ultima instancia 8 bits (1.2Vpp para el ampli)
            pre_out = (pre_out > 127 + 46) ? (127 + 46) : (pre_out);
            pre_out = (pre_out < 127 - 46) ? (127 - 46) : (pre_out);
            dac_output_voltage(DAC_CHANNEL, pre_out);
            current_time = esp_timer_get_time(); // Obtiene el tiempo actual en microsegundos
        } else if (ret == ESP_ERR_TIMEOUT) {
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            break;
        }
        
        // Feed esp watchdog
        // esp_wdt
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}