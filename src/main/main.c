#include <string.h>
#include "bno055.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* --------------------------------------------------------------------------
 * Configurazione I2C e parametri BNO055
 * -------------------------------------------------------------------------- */

#define I2C_MASTER_NUM              I2C_NUM_0

// Target: ESP32 classico (esp32doit-devkit-v1).
// Non usare GPIO6..GPIO11 (riservati alla flash SPI interna); su ESP32-S3 usare GPIO8/9.
#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21

// 100 kHz per tollerare jumper lunghi o pull-up deboli (passare a 400 kHz solo con pull-up esterni da 2.2-4.7 kOhm).
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       100
// Timeout hardware al massimo (~13 ms a 80 MHz APB) per attendere il clock-stretching del BNO055.
#define I2C_HW_TIMEOUT_MAX          0xFFFFF

#define BNO055_BOOT_DELAY_MS        800     // Tempo di avvio del BNO055 (~650 ms da datasheet)
#define BNO055_INIT_RETRIES         5
#define BNO055_INIT_RETRY_DELAY_MS  500
#define BNO055_CHIP_ID_VALUE        0xA0    // Valore atteso nel registro CHIP_ID (0x00)
#define BNO055_EULER_LSB_PER_DEG    16.0    // Scala Bosch: 1 LSB = 1/16 di grado

#define FCS_LOOP_PERIOD_MS          10      // Periodo loop di acquisizione (10 ms = 100 Hz)

static const char *TAG = "FCS_BNO055";
static struct bno055_t bno_sensor;

/* --------------------------------------------------------------------------
 * Callback richieste dal driver Bosch BNO055
 * -------------------------------------------------------------------------- */

// Lettura registri via I2C
static s8 bno055_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        dev_addr,
        &reg_addr, 1,
        reg_data, cnt,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

// Scrittura registri via I2C (il primo byte del buffer e' l'indirizzo del registro)
static s8 bno055_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    uint8_t buffer[cnt + 1];
    buffer[0] = reg_addr;
    if (cnt > 0) {
        memcpy(&buffer[1], reg_data, cnt);
    }

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        dev_addr,
        buffer, sizeof(buffer),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

// Attesa in millisecondi su FreeRTOS
static void bno055_delay_ms(u32 msec) {
    vTaskDelay(pdMS_TO_TICKS(msec));
}

/* --------------------------------------------------------------------------
 * Gestione e inizializzazione bus I2C
 * -------------------------------------------------------------------------- */

// Sonda un indirizzo I2C a 7 bit: ritorna ESP_OK se il dispositivo risponde con ACK
static esp_err_t i2c_probe_addr(u8 addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    i2c_cmd_link_delete(cmd);
    return err;
}

// Rileva l'indirizzo del BNO055: 0x28 (ADDR a GND) oppure 0x29 (ADDR a VDD)
static u8 bno055_detect_addr(void) {
    if (i2c_probe_addr(BNO055_I2C_ADDR1) == ESP_OK) {
        return BNO055_I2C_ADDR1;
    }
    if (i2c_probe_addr(BNO055_I2C_ADDR2) == ESP_OK) {
        return BNO055_I2C_ADDR2;
    }
    return 0;
}

// Sblocca il bus I2C se uno slave tiene SDA bassa (es. reset o brownout a meta' transazione)
static void i2c_bus_recovery(void) {
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    // Pull-up interni attivi per distinguere uno slave bloccato dall'assenza di pull-up esterni
    gpio_pullup_en(I2C_MASTER_SDA_IO);
    gpio_pullup_en(I2C_MASTER_SCL_IO);
    esp_rom_delay_us(200);

    if (gpio_get_level(I2C_MASTER_SDA_IO) == 0) {
        ESP_LOGW(TAG, "SDA tenuta bassa: eseguo bus recovery");

        // 9 impulsi di clock su SCL per far completare il byte allo slave e liberare SDA
        for (int i = 0; i < 9; i++) {
            gpio_set_level(I2C_MASTER_SCL_IO, 0);
            esp_rom_delay_us(5);
            gpio_set_level(I2C_MASTER_SCL_IO, 1);
            esp_rom_delay_us(5);
        }

        // Condizione di STOP manuale (fronte di salita su SDA con SCL alto)
        gpio_set_level(I2C_MASTER_SDA_IO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SDA_IO, 1);
        esp_rom_delay_us(5);
    }

    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
}

// Configura e installa il driver I2C Master
static esp_err_t i2c_master_init(void) {
    i2c_bus_recovery();

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

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_set_timeout(I2C_MASTER_NUM, I2C_HW_TIMEOUT_MAX);
}

/* --------------------------------------------------------------------------
 * Inizializzazione del sensore BNO055
 * -------------------------------------------------------------------------- */

static esp_err_t bno055_setup(void) {
    // Attende il completamento del boot interno prima della prima transazione I2C
    vTaskDelay(pdMS_TO_TICKS(BNO055_BOOT_DELAY_MS));

    u8 dev_addr = bno055_detect_addr();
    if (dev_addr == 0) {
        ESP_LOGE(TAG, "BNO055 non trovato sul bus I2C (SDA=GPIO%d, SCL=GPIO%d)",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return ESP_ERR_NOT_FOUND;
    }

    bno_sensor.bus_read = bno055_i2c_read;
    bno_sensor.bus_write = bno055_i2c_write;
    bno_sensor.delay_msec = bno055_delay_ms;
    bno_sensor.dev_addr = dev_addr;

    // Dopo un reset il chip puo' fare ACK sull'indirizzo prima che i registri siano pronti
    s8 comres = BNO055_ERROR;
    for (int attempt = 1; attempt <= BNO055_INIT_RETRIES; attempt++) {
        comres = bno055_init(&bno_sensor);
        if (comres == BNO055_SUCCESS) {
            break;
        }
        if (attempt < BNO055_INIT_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(BNO055_INIT_RETRY_DELAY_MS));
        }
    }

    if (comres != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Inizializzazione BNO055 fallita (codice %d)", comres);
        return ESP_FAIL;
    }

    if (bno_sensor.chip_id != BNO055_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Chip ID inatteso: 0x%02X (atteso 0x%02X)",
                 bno_sensor.chip_id, BNO055_CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Imposta alimentazione normale e fusione sensori a 9 gradi di liberta' (NDOF)
    if (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != BNO055_SUCCESS ||
        bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Configurazione modalita' BNO055 fallita");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BNO055 pronto (addr 0x%02X, chip ID 0x%02X, SW rev %d)",
             dev_addr, bno_sensor.chip_id, bno_sensor.sw_rev_id);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Entry point
 * -------------------------------------------------------------------------- */

void app_main(void) {
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Errore inizializzazione I2C Master: %s", esp_err_to_name(err));
        return;
    }

    if (bno055_setup() != ESP_OK) {
        return;
    }

    // Loop deterministico a 100 Hz per l'acquisizione degli angoli di Eulero
    struct bno055_euler_t euler_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FCS_LOOP_PERIOD_MS);

    while (1) {
        if (bno055_read_euler_hrp(&euler_data) == BNO055_SUCCESS) {
            double heading = (double)euler_data.h / BNO055_EULER_LSB_PER_DEG;
            double roll    = (double)euler_data.r / BNO055_EULER_LSB_PER_DEG;
            double pitch   = (double)euler_data.p / BNO055_EULER_LSB_PER_DEG;

            ESP_LOGI(TAG, "Euler -> Yaw/Heading: %6.2f° | Roll: %6.2f° | Pitch: %6.2f°",
                     heading, roll, pitch);
        } else {
            ESP_LOGW(TAG, "Lettura BNO055 fallita (I2C timeout o NACK)");
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
