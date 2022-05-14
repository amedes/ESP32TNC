/*
    BK4802 control program
*/
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
//#include "esp_timer.h"
#include "esp_log.h"
//#include "driver/dac.h"
//#include "driver/ledc.h"

#include "bk4802.h"

#define BK4802_TEST 1

#define BK4802_I2C_FREQ 400000
#define BK4802_I2C_TIMEOUT 10
#define BK4802_I2C_ADDR 0x48
#define BK4802_I2C_RETRY 3
#define BK4802_I2C_SEM 500
#define BK4802_I2C_TOUT_MAX 0xfffff

#define BK4802_QUEUE_LEN 8
#define BK4802_QUEUE_TIMEOUT 10
#define BK4802_TRX_WAIT 20

#define BK4802_DEFAULT_FREQ 431000000
#define BK4802_ASK 0        // 1: ASK enable, 0: ASK disable
#ifdef CONFIG_BK4802_PA_POWER
#define BK4802_TXPA_LEVEL CONFIG_BK4802_TX_POWER
#else
#define BK4802_TXPA_LEVEL 0 // Power level of TX PA, 0:-40dBm..7:12dBm
#endif
#define BK4802_REG_RSSI 24
#define BK4802_RX_VOL 15    // 0..15

static const char *TAG = "bk4802";

static const uint16_t bk4802_rxreg[BK4802_REG_MAX] = {
    0x511b, // REG0
    0x7dcc, // REG1
    0x0000, // REG2
    0,      // REG3
    0x0300, // REG4
    0x0c04, // REG5
    0xf140, // REG6
    0xed00, // REG7
    0x17e0  // REG8, ASK
        | (BK4802_ASK << 4),
    0xe0e0, // REG9
    0x8543, // REG10
    0x0700, // REG11
    0xa066, // REG12
    0xffff, // REG13
    0xffe0, // REG14
    0x061f, // REG15
    0x9e3c, // REG16
    0x1f00, // REG17
    0xd1c0, // REG18
    0x2000  // REG19, RX Volume
        | (BK4802_RX_VOL),
    0x01ff, // REG20
    0xe000, // REG21
    0x0040, // REG22
    0x18e0  // REG23, Power save off, ASK
        | (BK4802_ASK << 8),
};

static const uint16_t bk4802_txreg[BK4802_REG_MAX] = {
    0x5122, // REG0
    0x17da, // REG1
    0x0000, // REG2
    0,      // REG3
    0x7c00, // REG4
    0x0004, // REG5
    0xf140, // REG6
    0xed00, // REG7
    0x1700  // REG8, TX PA level, ASK
        | (BK4802_TXPA_LEVEL << 5)
        | (BK4802_ASK << 4),
    0xe0e0, // REG9
    0x8543, // REG10
    0x0700, // REG11
    0xa066, // REG12
    0xffff, // REG13
    0xffe0, // REG14
    0x061f, // REG15
    0x9e3c, // REG16
    0x1f00, // REG17
    0xd1d1, // REG18
    0x200f, // REG19
    0x01ff, // REG20
    0xe000, // REG21
    0x0340, // REG22
    0x98e0  // REG23, PTT, ASK
        | (1 << 9)  // PTT on for BK4802P?
        | (BK4802_ASK << 8),
};

static void bk4802_write(i2c_port_t i2c_num, int reg, int data)
{
    uint8_t buf[3] = { reg, data >> 8, data };

    i2c_set_timeout(i2c_num, BK4802_I2C_TOUT_MAX);

    esp_err_t err;
    for (int i = 0; i < BK4802_I2C_RETRY; i++) {
        err = i2c_master_write_to_device(i2c_num, BK4802_I2C_ADDR, &buf[0], sizeof(buf), BK4802_I2C_TIMEOUT / portTICK_PERIOD_MS);
        if (!err) break;
        ESP_LOGW(TAG, "i2c[%d] write fail %x, count = %d, reg = %d", i2c_num, err, i, reg);
    }
    if (err) ESP_LOGW(TAG, "i2c[%d] write fail %x", i2c_num, err);
}

static int bk4802_read(i2c_port_t i2c_num, int reg)
{
    uint8_t buf[2] = { reg };

    i2c_set_timeout(i2c_num, BK4802_I2C_TOUT_MAX);

    esp_err_t err;
    for (int i = 0; i < BK4802_I2C_RETRY; i++) {
        err = i2c_master_write_read_device(i2c_num, BK4802_I2C_ADDR, &buf[0], 1, &buf[0], sizeof(buf), BK4802_I2C_TIMEOUT / portTICK_PERIOD_MS);
        if (!err) break;
        ESP_LOGW(TAG, "i2c[%d] read fail %x, count = %d, reg = %d", i2c_num, err, i, reg);
    }
    if (err) ESP_LOGW(TAG, "i2c[%d] read fail %x", i2c_num, err);

    return (buf[0] << 8) | buf[1];
}

static void bk4802_set_reg(bk4802_t *bkp)
{
    for (int i = 4; i < BK4802_REG_MAX; i++) {
        bk4802_write(bkp->i2c_num, i, bkp->rxreg[i]);
    }

    for (int i = 2; i >=0; --i) {
        bk4802_write(bkp->i2c_num, i, bkp->rxreg[i]);
    }
}

static void bk4802_set_rx(bk4802_t *bkp)
{
    bkp->ptt = 0;

    if (xSemaphoreTake(bkp->i2c_sem, BK4802_I2C_SEM / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(TAG, "bk4802_set_rx() I2C semaphore take fail");
        return;
    }

    if (bkp->trx_pin >= 0) {
        gpio_set_level(bkp->trx_pin, 0); // PTT off
        ESP_LOGI(TAG, "TRX: pin = %d, level = %d", bkp->trx_pin, 0);
        vTaskDelay(BK4802_TRX_WAIT / portTICK_PERIOD_MS);
    }

    for (int i = 4; i < BK4802_REG_MAX; i++) {
        bk4802_write(bkp->i2c_num, i, bkp->rxreg[i]);
    }

    for (int i = 2; i >=0; --i) {
        bk4802_write(bkp->i2c_num, i, bkp->rxreg[i]);
    }

    xSemaphoreGive(bkp->i2c_sem);
}

void bk4802_set_tx(bk4802_t *bkp)
{
    bkp->ptt = 1;

    if (xSemaphoreTake(bkp->i2c_sem, BK4802_I2C_SEM / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(TAG, "bk4802_set_tx() I2C semaphore take fail");
        return;
    }

    if (bkp->trx_pin >= 0) {
        gpio_set_level(bkp->trx_pin, 1);  // PTT on
        ESP_LOGI(TAG, "TRX: pin = %d, level = %d", bkp->trx_pin, 1);
        vTaskDelay(BK4802_TRX_WAIT / portTICK_PERIOD_MS);
    }

    for (int i = 4; i < BK4802_REG_MAX; i++) {
        bk4802_write(bkp->i2c_num, i, bkp->txreg[i]);
    }

    for (int i = 2; i >=0; --i) {
        bk4802_write(bkp->i2c_num, i, bkp->txreg[i]);
    }

    xSemaphoreGive(bkp->i2c_sem);
}

#define MHZ (1000*1000)
#define BK4802_F_STEP 25
#define BK4802_F_DIV 53125

typedef struct {
    int freq_min;
    int freq_max;
    int fracn_base;
    int fracn_rem;
    int fracn_quotinet;
    int fracn_fraction;
    int if_base;
    int if_rem;
    int reg2_val;
} bk4802_freq_t;

static const bk4802_freq_t freq_table[] = {
    { 430*MHZ, 440*MHZ, 1357967601, -3125, 79, -2571, 432654, 42170, 0x0000, },
    { 144*MHZ, 146*MHZ, 1364283730, -36250, 237, -7713, 1297964, 20260, 0x2002, },
    { 50*MHZ, 54*MHZ, 1421128885, -15625, 711, -23139, 3893893, 7655, 0x8008, },
    { 28*MHZ, 297*MHZ/10, 1414812756, 17500, 1263, 11989, 6922476, 37220, 0xc00f, },
};

static int bk4802_calc_freq(int freq, uint16_t txreg[], uint16_t rxreg[])
{
    bk4802_freq_t const *p = &freq_table[0];

    for (int i = 0; i < sizeof(freq_table)/sizeof(freq_table[0]); i++) {

        if (freq >= p->freq_min && freq <= p->freq_max) {

            int fs = (freq - p->freq_min + BK4802_F_STEP/2) / BK4802_F_STEP;
            int fracn = p->fracn_base + p->fracn_quotinet * fs;
            int txfracn;
            int rxfracn;

            if (p->fracn_rem < 0) {
                uint fraction = -p->fracn_rem + (uint)-p->fracn_fraction * fs + BK4802_F_DIV/2;
                txfracn = fracn - fraction / BK4802_F_DIV;
                rxfracn = fracn - p->if_base - (fraction + p->if_rem) / BK4802_F_DIV;
            } else {
                int fraction = p->fracn_rem + p->fracn_fraction * fs + BK4802_F_DIV/2;
                txfracn = fracn + fraction / BK4802_F_DIV;
                rxfracn = fracn - p->if_base + (fraction - p->if_rem) / BK4802_F_DIV;
            }

            if (txreg) {
                txreg[0] = txfracn >> 16;
                txreg[1] = txfracn;
                txreg[2] = p->reg2_val;
            }

            if (rxreg) {
                rxreg[0] = rxfracn >> 16;
                rxreg[1] = rxfracn;
                rxreg[2] = p->reg2_val;
            }

            return 0;
        }

        p++;
    }

    return -1;
}

static void bk4802_set_freq(bk4802_t *bkp, int freq)
{
    if (xSemaphoreTake(bkp->i2c_sem, BK4802_I2C_SEM / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(TAG, "bk4802_set_freq() I2C semaphore take fail");
        return;
    }

    int rc = bk4802_calc_freq(freq, bkp->txreg, bkp->rxreg);

    if (rc) {
        ESP_LOGE(TAG, "freq error: %d", freq);
        xSemaphoreGive(bkp->i2c_sem);
        return;
    }

    bkp->freq = freq;
    uint16_t *reg = (bkp->ptt) ? &bkp->txreg[0] : &bkp->rxreg[0];

    for (int i = 2; i >= 0; i--) {
        bk4802_write(bkp->i2c_num, i, reg[i]);
    }

    xSemaphoreGive(bkp->i2c_sem);
}

#ifdef BK4802_TEST
typedef struct {
    int freq;
    int txfracn;
    int rxfracn;
} bk4802_freq_test_t;

static const bk4802_freq_test_t bk4802_test_table[] = {
    { 430*MHZ, 1357967601, 1357534946, },
    { 440*MHZ, 1389548243, 1389115588, },
    { 432999975, 1367441715, 1367009060, },
    { 144*MHZ, 1364283729, 1362985765, },
    { 146*MHZ, 1383232114, 1381934150, },
    { 144999975, 1373757685, 1372459721, },
    { 50*MHZ, 1421128885, 1417234992, },
    { 54*MHZ, 1534819195, 1530925302, },
    { 50999975, 1449550752, 1445656859, },
    { 28*MHZ, 1414812756, 1407890280, },
    { 297*MHZ/10, 1500712102, 1493789626, },
    { 28999975, 1465340520, 1458418043, },
};

static void bk4802_freq_test(void)
{
    bk4802_freq_test_t const *p = &bk4802_test_table[0];

    for (int i = 0; i < sizeof(bk4802_test_table)/sizeof(bk4802_test_table[0]); i++) {
        uint16_t txfreq[3];
        uint16_t rxfreq[3];
        assert(bk4802_calc_freq(p->freq, txfreq, rxfreq) == 0);
        uint txf = (txfreq[0] << 16) | txfreq[1];
        if (p->txfracn != txf) {
            ESP_LOGE(TAG, "freq = %d, fracn = %d, txfreq = %d", p->freq, p->txfracn, txf);
            abort();
        }
        uint rxf = (rxfreq[0] << 16) | rxfreq[1];
        if (p->rxfracn != rxf) {
            ESP_LOGE(TAG, "freq = %d, fracn = %d, rxfreq = %d", p->freq, p->rxfracn, rxf);
            abort();
        }
        p++;
    }
}
#endif

static void bk4802_rssi(bk4802_t *bkp)
{
    if (xSemaphoreTake(bkp->i2c_sem, 0) != pdTRUE) return;

    int rssi = bk4802_read(bkp->i2c_num, BK4802_REG_RSSI) & 0xff;

    if (rssi >= (bkp->rxreg[22] & 0xff)) {  // squelch open
        if (!bkp->squelch) {
            xSemaphoreTake(bkp->cdt_sem, 0);
            bkp->squelch = true;
            if (bkp->dcd_led >= 0) gpio_set_level(bkp->dcd_led, 1);
            ESP_LOGI(TAG, "squelch opened rssi = %02x", rssi);
        }
    } else {
        if (bkp->squelch) {
            xSemaphoreGive(bkp->cdt_sem);
            bkp->squelch = false;
            if (bkp->dcd_led >= 0) gpio_set_level(bkp->dcd_led, 0);
            ESP_LOGI(TAG, "squelch closed rssi = %02x", rssi);
        }
    }

    xSemaphoreGive(bkp->i2c_sem);
}

#ifdef BK4802_TEST
static void bk4802_reg_check(bk4802_t *bkp)
{
    for (int i = 0; i < BK4802_REG_MAX; i++) {
        int reg = bk4802_read(bkp->i2c_num, i);
        if (reg != bkp->rxreg[i]) {
            ESP_LOGE(TAG, "REG[%d] = %04x, %04x", i, reg, bkp->rxreg[i]);
            abort();
        }
    }
}

static void bk4802_reg_dump(bk4802_t *bkp)
{
    for (int i = 0; i < BK4802_REG_MAX; i++) {
        int reg = bk4802_read(bkp->i2c_num, i);
        ESP_LOGI(TAG, "REG[%d] = %04x", i, reg);
    }
}
#endif

typedef enum {
    BK_SET_TX = 1,
    BK_SET_RX,
    BK_SET_FREQ
} bk4802_mesgid_t;

typedef struct {
    bk4802_mesgid_t id;
    int val;
} bk4802_mesg_t;

static void bk4802_task(void *p)
{
    bk4802_t *bkp = (bk4802_t *)p;
    bk4802_mesg_t mesg;

    while (1) {
        if (xQueueReceive(bkp->queue, &mesg, BK4802_QUEUE_TIMEOUT / portTICK_PERIOD_MS) != pdTRUE) {
            // timeout
            if (!bkp->ptt) bk4802_rssi(bkp);

            continue;
        }

        switch (mesg.id) {
            case BK_SET_TX:
                bk4802_set_tx(bkp);
                break;

            case BK_SET_RX:
                bk4802_set_rx(bkp);
                break;

            case BK_SET_FREQ:
                bk4802_set_freq(bkp, mesg.val);
                break;

            default:
                ESP_LOGE(TAG, "Receive unknown mesg ID: %d", mesg.id);
        }
    }
}

void bk4802_ptt(bk4802_t *bkp, int on)
{
    bk4802_mesg_t mesg;
    mesg.id = on ? BK_SET_TX : BK_SET_RX;
    
    if (xQueueSend(bkp->queue, &mesg, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Send queue fail");
    } 
}

void bk4802_ptt_isr(bk4802_t *bkp, int on)
{
    bk4802_mesg_t mesg;
    mesg.id = on ? BK_SET_TX : BK_SET_RX;
    BaseType_t taskWoken = pdFALSE;
    
    if (xQueueSendFromISR(bkp->queue, &mesg, &taskWoken) != pdTRUE) {
        ESP_LOGE(TAG, "Send queue fail");
    }

    if (taskWoken) portYIELD_FROM_ISR();
}

void bk4802_freq(bk4802_t *bkp, int freq)
{
    bk4802_mesg_t mesg;
    mesg.id = BK_SET_FREQ;
    mesg.val = freq;

    if (xQueueSend(bkp->queue, &mesg, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Send queue fail");
    } 
}

void bk4802_init(bk4802_t *bkp)
{
#ifdef BK4802_TEST
    // check frequency calculation
    bk4802_freq_test();
#endif

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = bkp->sda_io_num,
        .scl_io_num = bkp->scl_io_num,
        //.sda_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,   // pullup by SR_T300
        .scl_pullup_en = GPIO_PULLUP_DISABLE,   // pullup by BK4802N?
        .master.clk_speed = BK4802_I2C_FREQ,
    };
    
    ESP_LOGI(TAG, "I2C port: %d", bkp->i2c_num);
    ESP_LOGI(TAG, "I2C SDA pin: %d", bkp->sda_io_num);
    ESP_LOGI(TAG, "I2C SCL pin: %d", bkp->scl_io_num);

    ESP_ERROR_CHECK(i2c_param_config(bkp->i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(bkp->i2c_num, conf.mode, 0, 0, 0));
    //ESP_ERROR_CHECK(i2c_set_pin(I2C_MASTER_PORT, I2C_MASTER_SDA, I2C_MASTER_SCL, true, true, I2C_MODE_MASTER));

    // semaphore for I2C
    bkp->i2c_sem = xSemaphoreCreateBinary();
    assert(bkp->i2c_sem != NULL);
    assert(xSemaphoreGive(bkp->i2c_sem) == pdTRUE);

    // semaphore for CDT
    bkp->cdt_sem = xSemaphoreCreateBinary();
    assert(bkp->cdt_sem != NULL);
    assert(xSemaphoreGive(bkp->cdt_sem) == pdTRUE);

    if (bkp->trx_pin >= 0) {
        ESP_LOGI(TAG, "I2C TRX pin: %d", bkp->trx_pin);
        gpio_reset_pin(bkp->trx_pin);
        gpio_set_direction(bkp->trx_pin, GPIO_MODE_DEF_OUTPUT);
    }
    if (bkp->dcd_led >= 0) {
        ESP_LOGI(TAG, "DCD LED pin: %d", bkp->dcd_led);
        gpio_reset_pin(bkp->dcd_led);
        gpio_set_direction(bkp->dcd_led, GPIO_MODE_DEF_OUTPUT);
    }

    for (int i = 0; i < BK4802_REG_MAX; i++) {
        bkp->rxreg[i] = bk4802_rxreg[i];
        bkp->txreg[i] = bk4802_txreg[i];
    }

#ifdef BK4802_TEST
    bk4802_reg_dump(bkp);
#endif

    bkp->ptt = false;
    bkp->squelch = false;

    // set default frequency
    if (!bkp->freq) bkp->freq = BK4802_DEFAULT_FREQ;
    ESP_LOGI(TAG, "frequency: %d Hz", bkp->freq);
    bk4802_set_freq(bkp, bkp->freq);

    bk4802_set_reg(bkp);

#ifdef BK4802_TEST
    bk4802_reg_check(bkp);
#endif

    // queue
    assert((bkp->queue = xQueueCreate(BK4802_QUEUE_LEN, sizeof(bk4802_mesg_t))) != NULL);

    // BK4802 task
    assert(xTaskCreatePinnedToCore(bk4802_task, "BK4802 task", 1024 * 4, bkp, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY) == pdPASS);
}

#if 0
void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    BaseType_t rc = xTaskCreate(i2c_slave_task, "I2C slave", 1024 * 4, NULL, tskIDLE_PRIORITY, NULL);
    if (rc != pdPASS) ESP_LOGE(TAG, "xTaskCreate fail");

    i2c_master_init();
    st7032_init();

    // TRX GPIO
    gpio_reset_pin(BK4802_TRX_PIN);
    gpio_set_direction(BK4802_TRX_PIN, GPIO_MODE_DEF_OUTPUT);

    // CDT GPIO
    gpio_reset_pin(BK4802_CDT_PIN);
    gpio_set_direction(BK4802_CDT_PIN, GPIO_MODE_DEF_OUTPUT);

    /*
    BaseType_t rc = xTaskCreate(bk4802_task, "BK4802 task", 1024 * 4, NULL, tskIDLE_PRIORITY, NULL);
    if (rc != pdPASS) ESP_LOGE(TAG, "xTaskCreate fail: bk4802_task");
    */

    st7032_print("Freq MHz");
    st7032_cmd(0xc0);
    st7032_print("431.020");

#if 0
    // test
    static const struct {
        int freq;
        uint32_t frac;
    } test_value[] = {
        { FREQ_430M, FRAC_430M },
        { FREQ_440M, FRAC_440M },
        { FREQ_433M, FRAC_433M },
        { 0, 0 },
    };

    /*
    printf("-7 / +2 = %d, -7 %% +2 = %d\n", -7 / 2, -7 % 2);
    printf("+7 / -2 = %d, +7 %% -2 = %d\n", 7 / -2, 7 % -2);
    printf("-7 / -2 = %d, -7 %% -2 = %d\n", -7 / -2, -7 % -2);
    */

    for (int i = 0; test_value[i].freq > 0; i++) {
        int freq = test_value[i].freq;
        uint32_t frac = test_value[i].frac;
        uint16_t txfreq[3];
        assert(bk4802_freq2(freq, txfreq, NULL) == 0);
        uint32_t bk4802_frac = (txfreq[0] << 16) | txfreq[1];
        printf("freq = %d, frac = %d, bk4802_frac = %d\n", freq, frac, bk4802_frac);
        assert(bk4802_frac == frac);
    }
#endif

    // bk4802_freq test
    bk4802_freq_test();

#define BK4802_DEFAULT_FREQ 431020000
//#define BK4802_DEFAULT_FREQ 144620000
//#define BK4802_DEFAULT_FREQ 52700000
//#define BK4802_DEFAULT_FREQ 29020000

#define BK4802_IF_FREQ 137000

    int freq = BK4802_DEFAULT_FREQ;
    ESP_ERROR_CHECK(bk4802_freq2(freq, txfreq, rxfreq));

    printf("TX freq = %d, frac_n = %04x %04x\n", BK4802_DEFAULT_FREQ, txfreq[0], txfreq[1]);
    printf("RX freq = %d, frac_n = %04x %04x\n", BK4802_DEFAULT_FREQ, txfreq[0], rxfreq[1]);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // dump all registers
    for (int i = 0; i <= 33; i++) {
        printf("REG[%d] = %04x\n", i, bk4802_read(i));
    }

    bk4802_rx_mode();

    // DAC CW generator
    static const dac_cw_config_t dac_cw_conf = {
        .en_ch = DAC_CHANNEL_1, // GPIO 25
        .scale = DAC_CW_SCALE_1,
        .phase = DAC_CW_PHASE_0,
        .freq = 1200,
        .offset = 1,
    };
    ESP_ERROR_CHECK(dac_cw_generator_config(&dac_cw_conf));
    ESP_ERROR_CHECK(dac_cw_generator_enable());
    ESP_ERROR_CHECK(dac_output_enable(dac_cw_conf.en_ch));

    // LEDC for ASK test signal
#define LEDC_OUTPUT_PIN GPIO_NUM_23
    static const ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 600,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    static const ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_PIN,
        .duty = ((1 << ledc_timer.duty_resolution) - 1) / 2,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    while (1) {

        printf("RX\n");
        bk4802_rx_mode();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        for (int i = 24; i <= 33; i++) {
            int data = bk4802_read(i);

            printf("reg[%d] = %04x\n", i, data);
        }

        int busy = 0;
        gpio_set_level(BK4802_CDT_PIN, 0);
        for (int i = 0; i < 1000; i++) {
            uint16_t rssi;

            if (bk4802_busy(&rssi) && !busy) {
                gpio_set_level(BK4802_CDT_PIN, 1);
                printf("channel busy: %04x\n", rssi);
                busy = 1;
            } if (busy && !bk4802_busy(&rssi)) {
                gpio_set_level(BK4802_CDT_PIN, 0);
                printf("channel clear: %04x\n", rssi);
                busy = 0;
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // check channel
        while (bk4802_busy(NULL)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        printf("TX %d MHz\n", freq);
        bk4802_tx_mode();
        gpio_set_level(BK4802_CDT_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BK4802_CDT_PIN, 0);

#if 0

#define MHZ (1000 * 1000)

        for (int freq = FREQ_430M; freq <= FREQ_440M; freq += BK4802_STEP) {
            snprintf((char *)lcd_ram, 17, "%03u.%04u%08x", freq / MHZ, (freq % MHZ) / 100, bk4802_freq(freq));        
            st7032_refresh();

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
#endif
    }

    printf("LCD init finished\n");

    vTaskDelay(portMAX_DELAY);
}
#endif
