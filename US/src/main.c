
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_err.h"

// ============================================================
// CONFIGURAZIONE
// ============================================================

#define TRIG_PIN GPIO_NUM_5
#define ECHO_PIN GPIO_NUM_18

#define SAMPLE_RATE_HZ          50
#define SAMPLE_PERIOD_MS        (1000 / SAMPLE_RATE_HZ)

#define TRIGGER_PULSE_US        10
#define ACQUISITION_TIMEOUT_US  15000

#define STATS_SAMPLES           15

#define MIN_DISTANCE_MM         200
#define MAX_DISTANCE_MM         600

#define RING_BUFFER_SIZE        32

static const char *TAG = "ULTRASONIC";

// ============================================================
// RISULTATI
// ============================================================

typedef enum
{
    STATUS_NO_ECHO = 0,
    STATUS_ECHO_STUCK_HIGH,
    STATUS_OUT_OF_RANGE,
    STATUS_VALID
} measurement_status_t;

typedef struct
{
    int64_t timestamp_us;
    uint32_t echo_us;
    uint32_t distance_mm;
    measurement_status_t status;
} measurement_t;

// ============================================================
// RING BUFFER
// ============================================================

static measurement_t measurement_ring[RING_BUFFER_SIZE];

static uint32_t ring_head = 0;
static uint32_t ring_count = 0;

static void ring_store(const measurement_t *record)
{
    measurement_ring[ring_head] = *record;

    ring_head = (ring_head + 1) % RING_BUFFER_SIZE;

    if (ring_count < RING_BUFFER_SIZE)
        ring_count++;
}

// ============================================================
// STATISTICHE
// ============================================================

static uint32_t attempts = 0;
static uint32_t triggers = 0;
static uint32_t valid = 0;
static uint32_t no_echo = 0;
static uint32_t stuck_high = 0;
static uint32_t out_of_range = 0;

static uint32_t last_echo_us = 0;
static uint32_t last_distance_mm = 0;

// ============================================================
// GPIO
// ============================================================

static void init_gpio(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << TRIG_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ECHO_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&trig));
    ESP_ERROR_CHECK(gpio_config(&echo));

    ESP_ERROR_CHECK(gpio_set_level(TRIG_PIN, 0));
}

// ============================================================
// ACQUISIZIONE ECHO SENZA RMT
// ============================================================

static measurement_status_t read_echo(
    uint32_t *echo_duration_us)
{
    *echo_duration_us = 0;

    // Genera impulso TRIG.
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(TRIGGER_PULSE_US);
    gpio_set_level(TRIG_PIN, 0);

    triggers++;

    // Timeout assoluto a partire dal trigger.
    int64_t deadline =
        esp_timer_get_time() + ACQUISITION_TIMEOUT_US;

    // --------------------------------------------------------
    // Attende il fronte di salita ECHO.
    // --------------------------------------------------------

    while (gpio_get_level(ECHO_PIN) == 0)
    {
        if (esp_timer_get_time() >= deadline)
        {
            return STATUS_NO_ECHO;
        }

        esp_rom_delay_us(1);
    }

    // Inizio impulso HIGH.
    int64_t echo_start = esp_timer_get_time();

    // --------------------------------------------------------
    // Attende il fronte di discesa ECHO.
    // --------------------------------------------------------

    while (gpio_get_level(ECHO_PIN) == 1)
    {
        if (esp_timer_get_time() >= deadline)
        {
            return STATUS_ECHO_STUCK_HIGH;
        }

        esp_rom_delay_us(1);
    }

    int64_t echo_end = esp_timer_get_time();

    *echo_duration_us =
        (uint32_t)(echo_end - echo_start);

    return STATUS_VALID;
}

// ============================================================
// STATISTICHE
// ============================================================

static void print_statistics(void)
{
    ESP_LOGI(TAG,
        "N=%lu | trig=%lu | valid=%lu | no_echo=%lu | stuck=%lu | out_range=%lu",
        (unsigned long)attempts,
        (unsigned long)triggers,
        (unsigned long)valid,
        (unsigned long)no_echo,
        (unsigned long)stuck_high,
        (unsigned long)out_of_range);

    ESP_LOGI(TAG,
        "Ultimo ECHO=%lu us | distanza=%lu mm | ring=%lu/%u",
        (unsigned long)last_echo_us,
        (unsigned long)last_distance_mm,
        (unsigned long)ring_count,
        (unsigned int)RING_BUFFER_SIZE);

    attempts = 0;
    triggers = 0;
    valid = 0;
    no_echo = 0;
    stuck_high = 0;
    out_of_range = 0;
}

// ============================================================
// TASK PRINCIPALE
// ============================================================

static void acquisition_loop(void)
{
    measurement_t record = {0};

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        attempts++;

        record = (measurement_t){0};
        record.timestamp_us = esp_timer_get_time();

        measurement_status_t status =
            read_echo(&record.echo_us);

        record.status = status;

        last_echo_us = record.echo_us;
        last_distance_mm = 0;

        if (status == STATUS_NO_ECHO)
        {
            no_echo++;
        }
        else if (status == STATUS_ECHO_STUCK_HIGH)
        {
            stuck_high++;
        }
        else
        {
            // Distanza in mm:
            // tempo_us * 343 / 2000

            record.distance_mm =
                (uint32_t)(((uint64_t)record.echo_us * 343ULL)
                           / 2000ULL);

            last_distance_mm = record.distance_mm;

            if (record.distance_mm < MIN_DISTANCE_MM ||
                record.distance_mm > MAX_DISTANCE_MM)
            {
                record.status = STATUS_OUT_OF_RANGE;
                out_of_range++;
            }
            else
            {
                record.status = STATUS_VALID;
                valid++;
            }
        }

        ring_store(&record);

        if (attempts >= STATS_SAMPLES)
        {
            print_statistics();
        }

        // Mantiene il periodo nominale di 20 ms.
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    init_gpio();

    ESP_LOGI(TAG, "GPIO polling mode - RMT disabilitato");
    ESP_LOGI(TAG, "TRIG GPIO%d | ECHO GPIO%d",
             TRIG_PIN, ECHO_PIN);

    ESP_LOGI(TAG, "Frequenza nominale: %d Hz",
             SAMPLE_RATE_HZ);

    acquisition_loop();
}
