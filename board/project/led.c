#include "led.h"

uint16_t led_cycle_duration = 500;
uint8_t led_cycles = 1;

void led_task()
{
    const uint WS2812_PIN = 16;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    ws2812_init(pio0, WS2812_PIN, 800000);

    while (1)
    {
        for (uint8_t i = led_cycles; i > 0; i--)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            put_pixel_rgb(0, 0, 100);
            vTaskDelay(led_cycle_duration / 2 / portTICK_PERIOD_MS);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            put_pixel_rgb(0, 0, 0);
            vTaskDelay(led_cycle_duration / 2 / portTICK_PERIOD_MS);
        }
        if (debug)
        {
            //printf("\nLed blink (%u): %u %u", uxTaskGetStackHighWaterMark(NULL), led_cycles, led_cycle_duration);
        }
        vTaskSuspend(NULL);
    }
}