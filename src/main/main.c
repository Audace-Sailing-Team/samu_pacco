#include <string.h>
#include "bno055.h"
#include "driver/i2c.h"
// #include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"


#define I2C_MASTER_NUM              I2C_NUM_0
// Target di build attuale: ESP32 "classico" (board = esp32doit-devkit-v1).
// ATTENZIONE: su ESP32 classico GPIO6..GPIO11 sono le linee del flash SPI interno (SPI0/SPI1):
// usarle per I2C manda in crash il chip (il core non riesce piu' a leggere le istruzioni dal flash).
// Su ESP32-S3 invece GPIO8/GPIO9 sono liberi e GPIO22 non esiste: se si passa a S3, usare 8/9.
#define I2C_MASTER_SCL_IO           GPIO_NUM_22  // SCL
#define I2C_MASTER_SDA_IO           GPIO_NUM_21  // SDA
// 100 kHz invece di 400 kHz: molto piu' tollerante con pull-up deboli / jumper lunghi.
// Da riportare a 400 kHz solo dopo aver verificato pull-up esterni da 2.2-4.7 kOhm.
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       100
#define BNO055_INIT_RETRIES         5
#define BNO055_CHIP_ID_VALUE        0xA0    // valore atteso nel registro CHIP_ID (0x00)

static const char *TAG = "FCS_BNO055";

static struct bno055_t bno_sensor;

static u32 i2c_error_count;

// Logga solo i primi errori e poi uno ogni tanto: il loop a 100 Hz altrimenti inonda la seriale
static void log_i2c_error(const char *op, u8 dev_addr, u8 reg_addr, esp_err_t err) {
    i2c_error_count++;
    if (i2c_error_count <= 8 || (i2c_error_count % 500) == 0) {
        ESP_LOGW(TAG, "I2C %s 0x%02X reg 0x%02X fallita: %s (errori totali: %lu)",
                 op, dev_addr, reg_addr, esp_err_to_name(err), (unsigned long)i2c_error_count);
    }
}

// Callback di lettura richiesta dal driver Bosch
s8 bno055_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        dev_addr,
        &reg_addr, 1,
        reg_data, cnt,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    if (err != ESP_OK) {
        log_i2c_error("read", dev_addr, reg_addr, err);
    }
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
    if (err != ESP_OK) {
        log_i2c_error("write", dev_addr, reg_addr, err);
    }
    return (err == ESP_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

// Callback di delay richiesta dal driver Bosch
void bno055_delay_ms(u32 msec) {
    vTaskDelay(pdMS_TO_TICKS(msec));
}

// Sonda l'indirizzo 7-bit: ritorna ESP_OK se il dispositivo risponde con ACK
static esp_err_t i2c_probe_addr(u8 addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Stampa tutti i dispositivi presenti sul bus: serve a distinguere un problema
// di cablaggio/pull-up da un problema di indirizzo del BNO055
static void i2c_scan_bus(void) {
    int found = 0;
    for (u8 addr = 0x08; addr < 0x78; addr++) {
        if (i2c_probe_addr(addr) == ESP_OK) {
            ESP_LOGI(TAG, "  dispositivo I2C trovato a 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  nessun dispositivo risponde sul bus I2C");
    }
}

// Il BNO055 puo' essere a 0x28 (ADDR a GND) o 0x29 (ADDR a VDD)
static u8 bno055_detect_addr(void) {
    if (i2c_probe_addr(BNO055_I2C_ADDR1) == ESP_OK) {
        return BNO055_I2C_ADDR1;
    }
    if (i2c_probe_addr(BNO055_I2C_ADDR2) == ESP_OK) {
        return BNO055_I2C_ADDR2;
    }
    return 0;
}

// Se uno slave ha lasciato SDA bassa (transazione interrotta, brownout, reset a metà),
// il master non riesce nemmeno a generare la START: ogni operazione va in timeout.
// 9 impulsi di clock su SCL + condizione di STOP liberano il bus.
static void i2c_bus_recovery(void) {
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    // Pull-up interni attivi durante la misura: se la linea legge 0 e' qualcuno che la tiene bassa
    // (slave bloccato o corto verso GND), non semplicemente l'assenza di pull-up esterni
    gpio_pullup_en(I2C_MASTER_SDA_IO);
    gpio_pullup_en(I2C_MASTER_SCL_IO);
    esp_rom_delay_us(200);

    ESP_LOGI(TAG, "Livelli a riposo: SDA=%d SCL=%d (attesi 1/1)",
             gpio_get_level(I2C_MASTER_SDA_IO), gpio_get_level(I2C_MASTER_SCL_IO));

    if (gpio_get_level(I2C_MASTER_SDA_IO) == 0) {
        ESP_LOGW(TAG, "SDA tenuta bassa: eseguo il bus recovery");
        for (int i = 0; i < 9; i++) {
            gpio_set_level(I2C_MASTER_SCL_IO, 0);
            esp_rom_delay_us(5);
            gpio_set_level(I2C_MASTER_SCL_IO, 1);
            esp_rom_delay_us(5);
        }
        gpio_set_level(I2C_MASTER_SDA_IO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SDA_IO, 1);
        esp_rom_delay_us(5);
        ESP_LOGI(TAG, "Livelli dopo il recovery: SDA=%d SCL=%d",
                 gpio_get_level(I2C_MASTER_SDA_IO), gpio_get_level(I2C_MASTER_SCL_IO));
    }
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
}

// Inizializzazione del bus I2C Master (compatibile con ESP-IDF v5.1 su PlatformIO)
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
    // Su ESP32 classico il BNO055 fa clock-stretching (tiene SCL basso) durante l'elaborazione interna.
    // Il timeout di default del controller hardware e' solo ~400 us (32000 cicli APB), che causa ESP_ERR_TIMEOUT.
    // Portando il timeout hardware al valore massimo consentito (0xFFFFF = ~13 ms a 80 MHz APB)
    // l'ESP32 aspetta che il BNO055 rilasci la linea SCL.
    return i2c_set_timeout(I2C_MASTER_NUM, 0xFFFFF);
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

    // 2. Il BNO055 impiega ~650 ms dal power-on prima di rispondere su I2C:
    //    senza questa attesa tutte le transazioni iniziali vanno in NACK
    vTaskDelay(pdMS_TO_TICKS(800));

    ESP_LOGI(TAG, "Scan del bus I2C...");
    i2c_scan_bus();

    u8 dev_addr = bno055_detect_addr();
    if (dev_addr == 0) {
        ESP_LOGE(TAG, "BNO055 non trovato sul bus: controllare alimentazione (3.3V), SDA->GPIO%d, "
                      "SCL->GPIO%d, i pull-up e il pin ADDR", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        return;
    }
    ESP_LOGI(TAG, "BNO055 presente all'indirizzo 0x%02X", dev_addr);

    // 3. Assegna le callback e l'indirizzo alla struttura del driver Bosch
    bno_sensor.bus_read = bno055_i2c_read;
    bno_sensor.bus_write = bno055_i2c_write;
    bno_sensor.delay_msec = bno055_delay_ms;
    bno_sensor.dev_addr = dev_addr;


    // 4. Inizializza il BNO055 (con retry: dopo un reset il chip puo' rispondere all'indirizzo
    //    ma non essere ancora pronto per le transazioni dati)
    s8 comres = BNO055_ERROR;
    for (int attempt = 1; attempt <= BNO055_INIT_RETRIES; attempt++) {
        comres = bno055_init(&bno_sensor);
        if (comres == BNO055_SUCCESS) {
            break;
        }
        ESP_LOGW(TAG, "Tentativo %d/%d di bno055_init fallito (codice %d)",
                 attempt, BNO055_INIT_RETRIES, comres);
        if (attempt < BNO055_INIT_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    if (comres != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Inizializzazione BNO055 fallita con codice di errore: %d", comres);
        ESP_LOGE(TAG, "Verifica collegamenti hardware (SDA/SCL) e indirizzo I2C (0x28 o 0x29)");
        return;
    }
    if (bno_sensor.chip_id != BNO055_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Chip ID inatteso: 0x%02X (atteso 0x%02X)", bno_sensor.chip_id, BNO055_CHIP_ID_VALUE);
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
