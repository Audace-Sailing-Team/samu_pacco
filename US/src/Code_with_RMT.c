// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>
// #include <limits.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"

// #include "driver/gpio.h"
// #include "driver/rmt_rx.h"

// #include "esp_timer.h"
// #include "esp_rom_sys.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_system.h"

// // ============================================================
// // CONFIGURAZIONE
// // ============================================================

// #define TRIG_PIN GPIO_NUM_5
// #define ECHO_PIN GPIO_NUM_18

// #define SAMPLE_RATE_HZ 50
// #define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)

// #define TRIGGER_PULSE_US 10

// #define RMT_RESOLUTION_HZ 1000000
// #define RMT_MEM_SYMBOLS 64

// // A 60 cm l'ECHO è circa 3.5 ms.
// // 5 ms lascia margine rispetto al campo operativo 20-60 cm.
// #define RMT_MAX_PULSE_NS 4500000
// #define RMT_MIN_PULSE_NS 1000

// // Deve rimanere inferiore al periodo di campionamento.
// #define ACQUISITION_TIMEOUT_MS 15

// // Frequenza di stampa delle statistiche.
// #define STATS_SAMPLES 15

// // Dimensione del buffer circolare in RAM.
// #define RING_BUFFER_SIZE 32

// // Campo operativo del test.
// #define MIN_DISTANCE_MM 200
// #define MAX_DISTANCE_MM 600

// // Test drift con riferimento esterno GPS PPS.
// // 0 = disabilitato.
// // 1 = abilitato: collegare un segnale PPS a PPS_PIN.
// #define ENABLE_PPS_DRIFT_TEST 0
// #define PPS_PIN GPIO_NUM_4

// // ============================================================

// static const char *TAG = "ULTRASONIC";

// static rmt_channel_handle_t rx_channel;
// static rmt_symbol_word_t symbols[RMT_MEM_SYMBOLS];

// static QueueHandle_t result_queue;
// static TaskHandle_t acquisition_task;
// static esp_timer_handle_t sample_timer;

// // Protegge il timestamp condiviso tra task e callback RMT.
// static portMUX_TYPE timestamp_mux = portMUX_INITIALIZER_UNLOCKED;
// static int64_t active_trigger_us = 0;

// // ============================================================
// // RISULTATI E STATI
// // ============================================================

// typedef enum
// {
//     MEAS_STATUS_START_ERROR = 0,
//     MEAS_STATUS_TIMEOUT,
//     MEAS_STATUS_NO_ECHO,
//     MEAS_STATUS_OUT_OF_RANGE,
//     MEAS_STATUS_VALID
// } measurement_status_t;

// typedef struct
// {
//     uint32_t echo_us;
//     uint32_t symbol_count;

//     int64_t trigger_us;
//     int64_t callback_us;
// } measurement_t;

// typedef struct
// {
//     uint32_t sequence;

//     int64_t timestamp_us;
//     int64_t callback_us;

//     uint32_t echo_us;
//     uint32_t distance_mm;

//     uint8_t status;
// } ring_record_t;

// // ============================================================
// // RING BUFFER
// // ============================================================

// // ring_head indica la prossima posizione da scrivere.
// // Quando il buffer è pieno, la scrittura sovrascrive il record
// // più vecchio. ring_count non supera mai RING_BUFFER_SIZE.

// static ring_record_t measurement_ring[RING_BUFFER_SIZE];

// static uint32_t ring_head = 0;
// static uint32_t ring_count = 0;
// static uint32_t ring_sequence = 0;
// static volatile uint32_t rmt_callback_count = 0;
// static volatile uint32_t rmt_zero_symbols = 0;
// static volatile uint32_t rmt_symbol_total = 0;
// static volatile uint32_t rmt_max_symbols = 0;
// static volatile uint32_t dbg_level0 = 0;
// static volatile uint32_t dbg_level1 = 0;
// static volatile uint32_t dbg_duration0 = 0;
// static volatile uint32_t dbg_duration1 = 0;
// static volatile int64_t dbg_callback_delay_us = 0;
// static volatile uint32_t dbg_queue_ok = 0;


// static void test_trig_gpio(void)
// {
//     while (1) {
//         gpio_set_level(TRIG_PIN, 1);
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         gpio_set_level(TRIG_PIN, 0);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }


// static void ring_store(const ring_record_t *record)
// {
//     measurement_ring[ring_head] = *record;

//     ring_head++;
//     if (ring_head >= RING_BUFFER_SIZE)
//     {
//         ring_head = 0;
//     }

//     if (ring_count < RING_BUFFER_SIZE)
//     {
//         ring_count++;
//     }
// }

// // Per leggere il buffer in ordine cronologico:
// // oldest = (ring_head + RING_BUFFER_SIZE - ring_count)
// //          % RING_BUFFER_SIZE
// //
// // Poi leggere ring_count elementi incrementando l'indice
// // con wrap modulo RING_BUFFER_SIZE.
// //
// // Il buffer è scritto esclusivamente dal task di acquisizione.

// // ============================================================
// // STATISTICHE
// // ============================================================

// static uint32_t attempts = 0;
// static uint32_t triggers = 0;
// static uint32_t completed = 0;
// static uint32_t valid = 0;
// static uint32_t invalid = 0;
// static uint32_t timeouts = 0;
// static uint32_t start_errors = 0;
// static uint32_t recovery_count = 0;
// static uint32_t recovery_errors = 0;
// static uint32_t missed_ticks = 0;
// static uint32_t callback_queue_drops = 0;

// static uint64_t interval_sum = 0;
// static uint64_t interval_min = UINT64_MAX;
// static uint64_t interval_max = 0;
// static uint32_t interval_count = 0;

// static int64_t last_trigger_us = 0;

// // ============================================================
// // DRIFT TEST: GPS PPS OPZIONALE
// // ============================================================

// #if ENABLE_PPS_DRIFT_TEST

// static portMUX_TYPE pps_mux = portMUX_INITIALIZER_UNLOCKED;

// static int64_t pps_first_us = 0;
// static int64_t pps_last_us = 0;
// static uint64_t pps_interval_sum = 0;

// static uint32_t pps_edges = 0;
// static uint32_t pps_intervals = 0;

// static void IRAM_ATTR pps_isr_handler(void *arg)
// {
//     int64_t now_us = esp_timer_get_time();

//     portENTER_CRITICAL_ISR(&pps_mux);

//     if (pps_edges == 0)
//     {
//         pps_first_us = now_us;
//     }
//     else
//     {
//         int64_t delta = now_us - pps_last_us;

//         if (delta > 0)
//         {
//             pps_interval_sum += (uint64_t)delta;
//             pps_intervals++;
//         }
//     }

//     pps_last_us = now_us;
//     pps_edges++;

//     portEXIT_CRITICAL_ISR(&pps_mux);
// }

// static void init_pps(void)
// {
//     gpio_config_t pps_config = {
//         .pin_bit_mask = 1ULL << PPS_PIN,
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_POSEDGE};

//     ESP_ERROR_CHECK(gpio_config(&pps_config));

//     esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

//     if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
//     {
//         ESP_ERROR_CHECK(err);
//     }

//     ESP_ERROR_CHECK(
//         gpio_isr_handler_add(PPS_PIN, pps_isr_handler, NULL));
// }

// static void print_pps_drift(void)
// {
//     uint64_t sum;
//     uint32_t intervals;
//     int64_t first;
//     int64_t last;

//     portENTER_CRITICAL(&pps_mux);

//     sum = pps_interval_sum;
//     intervals = pps_intervals;
//     first = pps_first_us;
//     last = pps_last_us;

//     portEXIT_CRITICAL(&pps_mux);

//     if (intervals == 0)
//     {
//         ESP_LOGI(TAG, "PPS: waiting for reference pulses");
//         return;
//     }

//     double average_us = (double)sum / intervals;

//     double ppm =
//         ((average_us - 1000000.0) / 1000000.0) * 1000000.0;

//     int64_t expected_us = (int64_t)intervals * 1000000LL;
//     int64_t measured_us = last - first;
//     int64_t cumulative_error_us = measured_us - expected_us;

//     ESP_LOGI(
//         TAG,
//         "PPS drift: avg=%.2f us | %.2f ppm | cumulative=%lld us",
//         average_us,
//         ppm,
//         (long long)cumulative_error_us);
// }

// #endif

// // ============================================================
// // RMT CALLBACK
// // ============================================================

// static bool IRAM_ATTR rmt_rx_callback(
//     rmt_channel_handle_t channel,
//     const rmt_rx_done_event_data_t *edata,
//     void *user_data)
// {
//     measurement_t result = {0};

//     // Il timestamp viene letto in sezione critica perché
//     // int64_t non è garantito atomico su ESP32 a 32 bit.

//     portENTER_CRITICAL_ISR(&timestamp_mux);
//     result.trigger_us = active_trigger_us;
//     portEXIT_CRITICAL_ISR(&timestamp_mux);

//     result.callback_us = esp_timer_get_time();

//     dbg_callback_delay_us =
//         result.callback_us - result.trigger_us;

//     if (edata->num_symbols > 0)
//     {
//         const rmt_symbol_word_t *s =
//             &edata->received_symbols[0];

//         dbg_level0 = s->level0;
//         dbg_level1 = s->level1;

//         dbg_duration0 = s->duration0;
//         dbg_duration1 = s->duration1;
//     }
//     result.symbol_count = edata->num_symbols;

//     rmt_callback_count++;

//     if (edata->num_symbols == 0)
//     {
//         rmt_zero_symbols++;
//     }

//     rmt_symbol_total += edata->num_symbols;

//     if (edata->num_symbols > rmt_max_symbols)
//     {
//         rmt_max_symbols = edata->num_symbols;
//     }

//     // RMT è configurato a 1 MHz: una unità di durata = 1 us.
//     // Si cerca il segmento HIGH più lungo.

//     for (size_t i = 0; i < edata->num_symbols; i++)
//     {

//         const rmt_symbol_word_t *s =
//             &edata->received_symbols[i];

//         if (s->level0 == 1 && s->duration0 > result.echo_us)
//         {
//             result.echo_us = s->duration0;
//         }

//         if (s->level1 == 1 && s->duration1 > result.echo_us)
//         {
//             result.echo_us = s->duration1;
//         }
//     }

//     BaseType_t task_woken = pdFALSE;

//     BaseType_t queue_result = xQueueSendFromISR(
//         result_queue,
//         &result,
//         &task_woken
//     );

//     if (queue_result == pdTRUE) {
//         dbg_queue_ok++;
//     } else {
//         callback_queue_drops++;
//     }

//     return task_woken == pdTRUE;
// }

// // ============================================================
// // TIMER CALLBACK
// // ============================================================

// static void periodic_timer_callback(void *arg)
// {
//     // Il callback del timer notifica il task.
//     // Nessuna acquisizione o stampa viene eseguita qui.

//     xTaskNotifyGive(acquisition_task);
// }

// // ============================================================
// // GPIO
// // ============================================================

// static void init_gpio(void)
// {
//     gpio_config_t trig = {
//         .pin_bit_mask = 1ULL << TRIG_PIN,
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE};

//     gpio_config_t echo = {
//         .pin_bit_mask = 1ULL << ECHO_PIN,
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type = GPIO_INTR_DISABLE};

//     ESP_ERROR_CHECK(gpio_config(&trig));
//     ESP_ERROR_CHECK(gpio_config(&echo));

//     ESP_ERROR_CHECK(gpio_set_level(TRIG_PIN, 0));
// }

// // ============================================================
// // RMT
// // ============================================================

// static void init_rmt(void)
// {
//     rmt_rx_channel_config_t config = {
//         .gpio_num = ECHO_PIN,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = RMT_RESOLUTION_HZ,
//         .mem_block_symbols = RMT_MEM_SYMBOLS,
//         .flags.invert_in = false};

//     ESP_ERROR_CHECK(
//         rmt_new_rx_channel(&config, &rx_channel));

//     rmt_rx_event_callbacks_t callbacks = {
//         .on_recv_done = rmt_rx_callback};

//     ESP_ERROR_CHECK(
//         rmt_rx_register_event_callbacks(
//             rx_channel,
//             &callbacks,
//             NULL));

//     ESP_ERROR_CHECK(rmt_enable(rx_channel));
// }

// // ============================================================
// // RECUPERO RMT
// // ============================================================

// static bool recover_rmt(void)
// {
//     esp_err_t err = rmt_disable(rx_channel);

//     if (err != ESP_OK)
//     {
//         recovery_errors++;
//         return false;
//     }

//     err = rmt_enable(rx_channel);

//     if (err != ESP_OK)
//     {
//         recovery_errors++;
//         return false;
//     }

//     recovery_count++;
//     return true;
// }

// // ============================================================
// // STATISTICHE
// // ============================================================

// static void print_statistics(void)
// {
//     double avg_interval_us = 0.0;
//     double actual_frequency = 0.0;

//     if (interval_count > 0)
//     {
//         avg_interval_us =
//             (double)interval_sum / interval_count;

//         if (avg_interval_us > 0.0)
//         {
//             actual_frequency =
//                 1000000.0 / avg_interval_us;
//         }
//     }

//     ESP_LOGI(
//         TAG,
//         "N=%lu | trig=%lu | done=%lu | valid=%lu | invalid=%lu | timeout=%lu",
//         (unsigned long)attempts,
//         (unsigned long)triggers,
//         (unsigned long)completed,
//         (unsigned long)valid,
//         (unsigned long)invalid,
//         (unsigned long)timeouts);

//     ESP_LOGI(
//         TAG,
//         "freq=%.2f Hz | interval avg/min/max=%.1f/%llu/%llu us | missed=%lu",
//         actual_frequency,
//         avg_interval_us,
//         (unsigned long long)(interval_count ? interval_min : 0),
//         (unsigned long long)interval_max,
//         (unsigned long)missed_ticks);

//     ESP_LOGI(
//         TAG,
//         "start_err=%lu | recoveries=%lu | recovery_err=%lu | queue_drop=%lu | ring=%lu/%u",
//         (unsigned long)start_errors,
//         (unsigned long)recovery_count,
//         (unsigned long)recovery_errors,
//         (unsigned long)callback_queue_drops,
//         (unsigned long)ring_count,
//         (unsigned int)RING_BUFFER_SIZE);

//     ESP_LOGI(
//         TAG,
//         "RMT callbacks=%lu | zero=%lu | symbols=%lu | max=%lu",
//         (unsigned long)rmt_callback_count,
//         (unsigned long)rmt_zero_symbols,
//         (unsigned long)rmt_symbol_total,
//         (unsigned long)rmt_max_symbols);

//     ESP_LOGI(
//         TAG,
//         "DBG delay=%lld us | L0=%lu D0=%lu | L1=%lu D1=%lu | queue_ok=%lu",
//         (long long)dbg_callback_delay_us,
//         (unsigned long)dbg_level0,
//         (unsigned long)dbg_duration0,
//         (unsigned long)dbg_level1,
//         (unsigned long)dbg_duration1,
//         (unsigned long)dbg_queue_ok);

// #if ENABLE_PPS_DRIFT_TEST
//     print_pps_drift();
// #else
//     ESP_LOGI(TAG, "PPS drift test disabled");
// #endif

//     // Reset delle statistiche di finestra.
//     // last_trigger_us viene mantenuto per non perdere
//     // la continuità nella misura degli intervalli.

//     attempts = 0;
//     triggers = 0;
//     completed = 0;
//     valid = 0;
//     invalid = 0;
//     timeouts = 0;
//     start_errors = 0;
//     recovery_count = 0;
//     recovery_errors = 0;
//     missed_ticks = 0;
//     callback_queue_drops = 0;

//     interval_sum = 0;
//     interval_count = 0;
//     interval_min = UINT64_MAX;
//     interval_max = 0;
// }

// // ============================================================
// // ACQUISITION TASK
// // ============================================================

// static void acquisition_loop(void)
// {
//     measurement_t result;

//     const rmt_receive_config_t receive_config = {
//         .signal_range_min_ns = RMT_MIN_PULSE_NS,
//         .signal_range_max_ns = RMT_MAX_PULSE_NS};

//     while (1)
//     {

//         // Attende il timer periodico.
//         // Le notifiche accumulate vengono consumate insieme:
//         // non vengono eseguiti campionamenti arretrati.

//         uint32_t pending =
//             ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         if (pending > 1)
//         {
//             missed_ticks += pending - 1;
//         }

//         attempts++;

//         ring_record_t record = {0};

//         record.sequence = ++ring_sequence;
//         record.timestamp_us = esp_timer_get_time();
//         record.status = MEAS_STATUS_START_ERROR;

//         // Elimina eventuali risultati rimasti in coda.
//         // Il ciclo precedente ha già completato o recuperato RMT.

//         xQueueReset(result_queue);

//         // Avvia prima RMT, poi genera TRIG.

//         esp_err_t err = rmt_receive(
//             rx_channel,
//             symbols,
//             sizeof(symbols),
//             &receive_config);

//         // Tentativo di recupero se il canale risulta disabilitato.

//         if (err == ESP_ERR_INVALID_STATE)
//         {

//             esp_err_t enable_err = rmt_enable(rx_channel);

//             if (enable_err == ESP_OK)
//             {
//                 recovery_count++;

//                 err = rmt_receive(
//                     rx_channel,
//                     symbols,
//                     sizeof(symbols),
//                     &receive_config);
//             }
//             else
//             {
//                 recovery_errors++;
//             }
//         }

//         if (err != ESP_OK)
//         {

//             start_errors++;

//             record.status = MEAS_STATUS_START_ERROR;
//             ring_store(&record);

//             goto cycle_end;
//         }

//         // Timestamp immediatamente prima del fronte TRIG.
//         // La sezione critica evita letture parziali nel callback.

//         int64_t trigger_time = esp_timer_get_time();

//         portENTER_CRITICAL(&timestamp_mux);
//         active_trigger_us = trigger_time;
//         portEXIT_CRITICAL(&timestamp_mux);

//         record.timestamp_us = trigger_time;

//         gpio_set_level(TRIG_PIN, 1);
//         esp_rom_delay_us(TRIGGER_PULSE_US);
//         gpio_set_level(TRIG_PIN, 0);

//         triggers++;

//         // Intervallo effettivo tra i trigger.

//         if (last_trigger_us != 0)
//         {

//             uint64_t interval =
//                 (uint64_t)(trigger_time - last_trigger_us);

//             interval_sum += interval;
//             interval_count++;

//             if (interval < interval_min)
//             {
//                 interval_min = interval;
//             }

//             if (interval > interval_max)
//             {
//                 interval_max = interval;
//             }
//         }

//         last_trigger_us = trigger_time;

//         // Attesa del completamento RMT.

//         if (xQueueReceive(
//                 result_queue,
//                 &result,
//                 pdMS_TO_TICKS(ACQUISITION_TIMEOUT_MS)) != pdTRUE)
//         {

//             timeouts++;

//             record.status = MEAS_STATUS_TIMEOUT;

//             // Interrompe la ricezione rimasta attiva e
//             // prepara il canale per il ciclo successivo.

//             recover_rmt();

//             // Un eventuale callback tardivo non deve essere
//             // associato alla misura successiva.

//             xQueueReset(result_queue);

//             ring_store(&record);

//             goto cycle_end;
//         }

//         completed++;

//         record.callback_us = result.callback_us;
//         record.echo_us = result.echo_us;

//         if (result.echo_us == 0)
//         {

//             invalid++;
//             record.status = MEAS_STATUS_NO_ECHO;

//             ring_store(&record);

//             goto cycle_end;
//         }

//         // Distanza in millimetri:
//         // distanza = tempo * velocità del suono / 2.
//         //
//         // 343 m/s = 343 mm/ms.
//         // Con tempo in us: echo_us * 343 / 2000.

//         uint32_t distance_mm =
//     (uint32_t)(((uint64_t)result.echo_us * 343ULL) / 2000ULL);

//         record.distance_mm = distance_mm;

//         if (distance_mm < MIN_DISTANCE_MM ||
//             distance_mm > MAX_DISTANCE_MM)
//         {

//             invalid++;
//             record.status = MEAS_STATUS_OUT_OF_RANGE;

//             ring_store(&record);

//             goto cycle_end;
//         }

//         valid++;
//         record.status = MEAS_STATUS_VALID;

//         ring_store(&record);

//     cycle_end:

//         // Il controllo delle statistiche è unico e viene
//         // eseguito anche dopo timeout ed errori.

//         if (attempts >= STATS_SAMPLES)
//         {
//             print_statistics();
//         }
//     }
// }

// // ============================================================
// // APP MAIN
// // ============================================================

// void app_main(void)
// {
//     init_gpio();

// #if ENABLE_PPS_DRIFT_TEST
//     init_pps();
// #endif

//     result_queue = xQueueCreate(
//         1,
//         sizeof(measurement_t));

//     if (result_queue == NULL)
//     {
//         ESP_LOGE(TAG, "Queue allocation failed");
//         return;
//     }

//     init_rmt();

//     acquisition_task = xTaskGetCurrentTaskHandle();

//     const esp_timer_create_args_t timer_args = {
//         .callback = periodic_timer_callback,
//         .arg = NULL,
//         .dispatch_method = ESP_TIMER_TASK,
//         .name = "sample_timer"};

//     ESP_ERROR_CHECK(
//         esp_timer_create(&timer_args, &sample_timer));

//     ESP_ERROR_CHECK(
//         esp_timer_start_periodic(
//             sample_timer,
//             SAMPLE_PERIOD_US));

//     ESP_LOGI(
//         TAG,
//         "Started | target=%d Hz | period=%d us | RMT=%d MHz | ring=%u",
//         SAMPLE_RATE_HZ,
//         SAMPLE_PERIOD_US,
//         RMT_RESOLUTION_HZ / 1000000,
//         (unsigned int)RING_BUFFER_SIZE);

//     acquisition_loop();
// }
