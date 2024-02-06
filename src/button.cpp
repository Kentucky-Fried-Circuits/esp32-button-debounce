#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <vector>

#include "button.h"

#define TAG "BUTTON"

typedef struct
{
    unsigned long long pin;
    bool inverted;
    uint16_t history;
    uint32_t down_time;
    uint32_t next_long_time;
} debounce_t;

int pin_count = 0;
debounce_t *debounce;
QueueHandle_t queue;
std::vector<uint8_t> vec;

static void update_button(debounce_t *d)
{
    d->history = (d->history << 1) | gpio_get_level((gpio_num_t )d->pin);
}

#define MASK 0b1111000000111111
static bool button_rose(debounce_t *d)
{
    if ((d->history & MASK) == 0b0000000000111111)
    {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d)
{
    if ((d->history & MASK) == 0b1111000000000000)
    {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d)
{
    if (d->inverted)
        return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d)
{
    if (d->inverted)
        return button_rose(d);
    return button_fell(d);
}

static uint32_t millis()
{
    return esp_timer_get_time() / 1000;
}

static void send_event(debounce_t db, int ev)
{
    button_event_t event = {
        .pin = db.pin,
        .event = ev,
    };
    xQueueSend(queue, &event, 100);
}

static void button_task(void *pvParameter)
{
    for (;;)
    {
        for (int idx = 0; idx < pin_count; idx++)
        {
            update_button(&debounce[idx]);
            if (button_up(&debounce[idx]))
            {
                debounce[idx].down_time = 0;
                ESP_LOGI(TAG, "%llu UP", debounce[idx].pin);
                send_event(debounce[idx], BUTTON_UP);
            }
            else if (debounce[idx].down_time && millis() >= debounce[idx].next_long_time)
            {
                ESP_LOGI(TAG, "%llu LONG", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].next_long_time + CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS;
                send_event(debounce[idx], BUTTON_HELD);
            }
            else if (button_down(&debounce[idx]) && debounce[idx].down_time == 0)
            {
                debounce[idx].down_time = millis();
                ESP_LOGI(TAG, "%llu DOWN", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].down_time + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
                send_event(debounce[idx], BUTTON_DOWN);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// QueueHandle_t button_init(unsigned long long pin_select)
// {
//     return pulled_button_init(pin_select, GPIO_FLOATING);
// }

bool pulled_button_init(unsigned long long pin_select, gpio_pull_mode_t pull_mode)
{
    // Scan the pin map to determine number of pins
    for (int pin = 0; pin <= 39; pin++)
    {
        if ((1ULL << pin) & pin_select)
        {
            pin_count++;
            vec.push_back(pin_select);

            // Configure the pins
            gpio_config_t io_conf;
            io_conf.intr_type = GPIO_INTR_NEGEDGE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            io_conf.pin_bit_mask = (1ULL << pin_select);
            gpio_config(&io_conf);

            return true;
        }
    }
    return false;
}

QueueHandle_t button_begin()
{
    // Initialize global state and queue
    debounce = (debounce_t*)calloc(pin_count, sizeof(debounce_t));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (auto &it : vec)
    {
        ESP_LOGI(TAG, "Registering button input: %d", it);
        debounce[idx].pin = it;
        debounce[idx].down_time = 0;
        debounce[idx].inverted = true;
        if (debounce[idx].inverted)
            debounce[idx].history = 0xffff;
        idx++;
    }

    queue = xQueueCreate(CONFIG_ESP32_BUTTON_QUEUE_SIZE, sizeof(button_event_t));
    // Spawn a task to monitor the pins
    xTaskCreate(&button_task, "button_task", CONFIG_ESP32_BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);

    return queue;
}