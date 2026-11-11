
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

#define SAMPLE_RATE_HZ          5
#define SAMPLE_PERIOD_MS        (1000 / SAMPLE_RATE_HZ)

#define TRIGGER_PULSE_US        10
#define ACQUISITION_TIMEOUT_US  45000

#define STATS_SAMPLES           5

#define MIN_DISTANCE_MM         1
#define MAX_DISTANCE_MM         200

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

            record.status = STATUS_VALID;
            valid++;
        }

        ring_store(&record);

        ESP_LOGI(TAG,
    "ECHO: %lu us | DISTANZA: %lu mm | STATUS: %d",
    (unsigned long)record.echo_us,
    (unsigned long)record.distance_mm,
    record.status);

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



//TEST 200 HZ

// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "driver/gpio.h"
// #include "esp_timer.h"
// #include "esp_rom_sys.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_task_wdt.h"

// // ============================================================
// // CONFIGURAZIONE TEST 
// // ============================================================

// #define TRIG_PIN                GPIO_NUM_5
// #define ECHO_PIN                GPIO_NUM_18

// // Nota: 200 Hz (5ms) sono estremi per HC-SR04. Se vedi molti NoEcho, 
// // prova a impostare SAMPLE_RATE_HZ a 20 o 50 Hz.
// #define SAMPLE_RATE_HZ          200
// #define SAMPLE_PERIOD_US        (1000000 / SAMPLE_RATE_HZ) 

// #define TRIGGER_PULSE_US        15
// #define ACQUISITION_TIMEOUT_US  4500

// #define STATS_SAMPLES           200
// #define STUCK_HIGH_RECOVERY_LIMIT 10  

// #define MIN_DISTANCE_MM         200
// #define MAX_DISTANCE_MM         600

// static const char *TAG = "ULTRASONIC";

// // ============================================================
// // STATI
// // ============================================================

// typedef enum
// {
//     STATUS_NO_ECHO = 0,
//     STATUS_STUCK_HIGH,
//     STATUS_OUT_OF_RANGE,
//     STATUS_VALID
// } measurement_status_t;

// // ============================================================
// // STATISTICHE
// // ============================================================

// static uint32_t attempts = 0;
// static uint32_t valid = 0;
// static uint32_t no_echo = 0;
// static uint32_t stuck_high = 0;
// static uint32_t out_of_range = 0;
// static uint32_t missed_periods = 0;
// static uint32_t consecutive_stuck = 0;

// static uint32_t last_echo_us = 0;
// static uint32_t last_distance_mm = 0;

// static uint32_t min_period_us = UINT32_MAX;
// static uint32_t max_period_us = 0;
// static uint64_t sum_period_us = 0;

// static uint32_t min_echo_us = UINT32_MAX;
// static uint32_t max_echo_us = 0;

// static int64_t previous_trigger_us = 0;

// // ============================================================
// // GPIO & RECOVERY
// // ============================================================

// static void init_gpio(void)
// {
//     gpio_config_t trig = {
//         .pin_bit_mask = 1ULL << TRIG_PIN,
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };

//     gpio_config_t echo = {
//         .pin_bit_mask = 1ULL << ECHO_PIN,
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };

//     ESP_ERROR_CHECK(gpio_config(&trig));
//     ESP_ERROR_CHECK(gpio_config(&echo));
//     ESP_ERROR_CHECK(gpio_set_level(TRIG_PIN, 0));
// }

// static void reset_echo_pin_state(void)
// {
//     ESP_LOGW(TAG, "Tentativo di reset hardware/software del pin ECHO...");
//     gpio_set_direction(ECHO_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(ECHO_PIN, 0);
//     esp_rom_delay_us(50);
//     gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
// }

// // ============================================================
// // ACQUISIZIONE ECHO
// // ============================================================

// static measurement_status_t read_echo(
//     uint32_t *echo_duration_us,
//     uint32_t *trigger_period_us)
// {
//     *echo_duration_us = 0;
//     *trigger_period_us = 0;

//     if (gpio_get_level(ECHO_PIN) == 1)
//     {
//         return STATUS_STUCK_HIGH;
//     }

//     // Genera TRIG.
//     gpio_set_level(TRIG_PIN, 1);
//     esp_rom_delay_us(TRIGGER_PULSE_US);
//     gpio_set_level(TRIG_PIN, 0);

//     int64_t trigger_time = esp_timer_get_time();

//     if (previous_trigger_us != 0)
//     {
//         *trigger_period_us = (uint32_t)(trigger_time - previous_trigger_us);
//     }
//     previous_trigger_us = trigger_time;

//     int64_t deadline = trigger_time + ACQUISITION_TIMEOUT_US;

//     while (gpio_get_level(ECHO_PIN) == 0)
//     {
//         if (esp_timer_get_time() >= deadline)
//             return STATUS_NO_ECHO;
//     }

//     int64_t echo_start = esp_timer_get_time();

//     while (gpio_get_level(ECHO_PIN) == 1)
//     {
//         if (esp_timer_get_time() >= deadline)
//             return STATUS_STUCK_HIGH;
//     }

//     int64_t echo_end = esp_timer_get_time();
//     *echo_duration_us = (uint32_t)(echo_end - echo_start);

//     return STATUS_VALID;
// }

// // ============================================================
// // STATISTICHE
// // ============================================================

// static void print_statistics(void)
// {
//     uint32_t received = valid + out_of_range;
//     uint32_t received_total = received + stuck_high;

//     uint32_t average_period = attempts > 1 ? (uint32_t)(sum_period_us / (attempts - 1)) : 0;
//     float actual_hz = average_period > 0 ? 1000000.0f / average_period : 0.0f;
//     float valid_percent = attempts > 0 ? (100.0f * valid / attempts) : 0.0f;
//     float echo_percent = attempts > 0 ? (100.0f * received_total / attempts) : 0.0f;

//     ESP_LOGI(TAG, "Tentativi=%lu | validi=%lu (%.1f%%) | ECHO=%.1f%%",
//              (unsigned long)attempts, (unsigned long)valid, valid_percent, echo_percent);

//     ESP_LOGI(TAG, "NoEcho=%lu | Stuck=%lu | FuoriRange=%lu | Periodi saltati=%lu",
//              (unsigned long)no_echo, (unsigned long)stuck_high,
//              (unsigned long)out_of_range, (unsigned long)missed_periods);

//     ESP_LOGI(TAG, "TRIG: medio=%lu us | min=%lu us | max=%lu us | frequenza=%.1f Hz",
//              (unsigned long)average_period, (unsigned long)min_period_us,
//              (unsigned long)max_period_us, actual_hz);

//     ESP_LOGI(TAG, "Ultimo ECHO=%lu us | distanza=%lu mm",
//              (unsigned long)last_echo_us, (unsigned long)last_distance_mm);

//     if (min_echo_us != UINT32_MAX)
//     {
//         ESP_LOGI(TAG, "Durata ECHO: min=%lu us | max=%lu us",
//                  (unsigned long)min_echo_us, (unsigned long)max_echo_us);
//     }

//     ESP_LOGI(TAG, "--------------------------------");

//     attempts = 0;
//     valid = 0;
//     no_echo = 0;
//     stuck_high = 0;
//     out_of_range = 0;
//     missed_periods = 0;
//     min_period_us = UINT32_MAX;
//     max_period_us = 0;
//     sum_period_us = 0;
//     min_echo_us = UINT32_MAX;
//     max_echo_us = 0;
// }

// // ============================================================
// // TASK DEDICATO DI ACQUISIZIONE
// // ============================================================

// static void ultrasonic_task(void *pvParameters)
// {
//     // Registra questo task nel Task Watchdog Timer
//     ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

//     int64_t next_trigger_us = esp_timer_get_time();

//     while (1)
//     {
//         int64_t now = esp_timer_get_time();

//         if (now < next_trigger_us)
//         {
//             int64_t remaining = next_trigger_us - now;
//             if (remaining > 2000)
//             {
//                 vTaskDelay(pdMS_TO_TICKS(1));
//             }
//             else if (remaining > 100)
//             {
//                 esp_rom_delay_us(remaining - 50);
//             }
            
//             while (esp_timer_get_time() < next_trigger_us) {
//                 // Attesa attiva fine
//             }
//         }

//         uint32_t echo_us = 0;
//         uint32_t period_us = 0;

//         measurement_status_t status = read_echo(&echo_us, &period_us);
//         attempts++;

//         last_echo_us = echo_us;
//         last_distance_mm = 0;

//         if (status == STATUS_STUCK_HIGH)
//         {
//             stuck_high++;
//             consecutive_stuck++;
//             if (consecutive_stuck >= STUCK_HIGH_RECOVERY_LIMIT)
//             {
//                 reset_echo_pin_state();
//                 consecutive_stuck = 0;
//             }
//         }
//         else
//         {
//             consecutive_stuck = 0;
//         }

//         if (period_us > 0)
//         {
//             if (period_us < min_period_us) min_period_us = period_us;
//             if (period_us > max_period_us) max_period_us = period_us;
//             sum_period_us += period_us;

//             if (period_us > SAMPLE_PERIOD_US + 200)
//                 missed_periods++;
//         }

//         if (status == STATUS_NO_ECHO)
//         {
//             no_echo++;
//         }
//         else if (status == STATUS_VALID)
//         {
//             if (echo_us < min_echo_us) min_echo_us = echo_us;
//             if (echo_us > max_echo_us) max_echo_us = echo_us;

//             uint32_t distance_mm = (uint32_t)(((uint64_t)echo_us * 343ULL) / 2000ULL);
//             last_distance_mm = distance_mm;

//             if (distance_mm < MIN_DISTANCE_MM || distance_mm > MAX_DISTANCE_MM)
//             {
//                 out_of_range++;
//             }
//             else
//             {
//                 valid++;
//             }
//         }

//         next_trigger_us += SAMPLE_PERIOD_US;
//         now = esp_timer_get_time();
        
//         if (now >= next_trigger_us)
//         {
//             next_trigger_us = now + SAMPLE_PERIOD_US;
//         }

//         if (attempts >= STATS_SAMPLES)
//         {
//             print_statistics();
//         }

//         // Nutre il watchdog a ogni singola iterazione per evitare timeout su IDLE1
//         esp_task_wdt_reset();
//     }
// }

// // ============================================================
// // APP MAIN
// // ============================================================

// void app_main(void)
// {
//     init_gpio();

//     ESP_LOGI(TAG, "====================================");
//     ESP_LOGI(TAG, "TEST FREQUENZA ULTRASONICO (STABILE)");
//     ESP_LOGI(TAG, "Frequenza richiesta: %d Hz", SAMPLE_RATE_HZ);
//     ESP_LOGI(TAG, "Periodo richiesto: %d us", SAMPLE_PERIOD_US);
//     ESP_LOGI(TAG, "Timeout ECHO: %d us", ACQUISITION_TIMEOUT_US);
//     ESP_LOGI(TAG, "TRIG GPIO%d | ECHO GPIO%d", TRIG_PIN, ECHO_PIN);
//     ESP_LOGI(TAG, "====================================");

//     // Priorità abbassata a 1 (invece di 5) per evitare di bloccare il task IDLE della CPU 1
//     xTaskCreatePinnedToCore(
//         ultrasonic_task,
//         "ultrasonic_task",
//         4096,
//         NULL,
//         1, 
//         NULL,
//         1
//     );
// }