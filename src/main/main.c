#include <string.h>
#include "bno055.h"
#include "driver/i2c.h"
// #include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"


#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           GPIO_NUM_9   // Pin SCL per ESP32-S3 (o altro GPIO libero; GPIO 22 non esiste su ESP32-S3)
#define I2C_MASTER_SDA_IO           GPIO_NUM_8   // Pin SDA per ESP32-S3
#define I2C_MASTER_FREQ_HZ          400000       // 400 kHz Fast Mode
#define I2C_MASTER_TIMEOUT_MS       50
#define BNO055_DEV_ADDR             BNO055_I2C_ADDR1 // 0x28 (oppure BNO055_I2C_ADDR2 = 0x29)

static const char *TAG = "FCS_BNO055";

static struct bno055_t bno_sensor;

// Callback di lettura richiesta dal driver Bosch
s8 bno055_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        dev_addr,
        &reg_addr, 1,
        reg_data, cnt,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

// Callback di scrittura richiesta dal driver Bosch
s8 bno055_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    uint8_t buffer[cnt + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, cnt);

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        dev_addr,
        buffer, sizeof(buffer),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

// Callback di delay richiesta dal driver Bosch
void bno055_delay_ms(u32 msec) {
    vTaskDelay(pdMS_TO_TICKS(msec));
}

// Inizializzazione del bus I2C Master (compatibile con ESP-IDF v5.1 su PlatformIO)
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) {
    ESP_LOGI(TAG, "Avvio Flight Control System - Inizializzazione BNO055...");

    // 1. Inizializza il bus I2C su ESP32
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore inizializzazione I2C Master: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Bus I2C Master configurato con successo (SDA: GPIO %d, SCL: GPIO %d)", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // 2. Assegna le callback e l'indirizzo alla struttura del driver Bosch
    bno_sensor.bus_read = bno055_i2c_read;
    bno_sensor.bus_write = bno055_i2c_write;
    bno_sensor.delay_msec = bno055_delay_ms;
    bno_sensor.dev_addr = BNO055_DEV_ADDR;

    // 3. Inizializza il BNO055
    s8 comres = bno055_init(&bno_sensor);
    if (comres != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Inizializzazione BNO055 fallita con codice di errore: %d", comres);
        ESP_LOGE(TAG, "Verifica collegamenti hardware (SDA/SCL) e indirizzo I2C (0x28 o 0x29)");
        return;
    }
    ESP_LOGI(TAG, "BNO055 rilevato! Chip ID: 0x%02X, SW Rev: %d", 
             bno_sensor.chip_id, bno_sensor.sw_rev_id);

    // 4. Configura la modalità di alimentazione normale e la modalità NDOF (9-DOF Fusion)
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    ESP_LOGI(TAG, "Sensore configurato in modalita' NDOF. Avvio loop a 100 Hz...");

    // 5. Loop di acquisizione ad alta frequenza (100 Hz = intervallo 10 ms per FCS)
    struct bno055_euler_t euler_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10); // 10 ms = 100 Hz

    while (1) {
        // Lettura angoli di Eulero (Heading, Roll, Pitch)
        if (bno055_read_euler_hrp(&euler_data) == BNO055_SUCCESS) {
            // Nel driver Bosch, il dato raw di Eulero ha scala 1 LSB = 1/16 di grado (16 LSB/degree)
            double heading = (double)euler_data.h / 16.0;
            double roll    = (double)euler_data.r / 16.0;
            double pitch   = (double)euler_data.p / 16.0;

            ESP_LOGI(TAG, "Euler -> Yaw/Heading: %6.2f° | Roll: %6.2f° | Pitch: %6.2f°", 
                     heading, roll, pitch);
        } else {
            ESP_LOGW(TAG, "Lettura BNO055 fallita (I2C timeout o NACK)");
        }

        // Delay deterministico per mantenere i 100 Hz precisi senza drift
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}
