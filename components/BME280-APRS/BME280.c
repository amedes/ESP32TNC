/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 2021.02.24      ESP32 BME280 try  */
/* 2021.03.04      Cleaned .c file   */
/* 2021.03.05      Add               */
/* 2021.03.11      Clean up          */

#include "config.h"
#include "fx25.h"     /* for fx25_send_packet() */
#include "BME280.h"   /* for BE280 test */

#define PKT_LEN 1024
#define SSID 0x60
#define UI_CONTROL 0x03
#define UI_PID 0xf0


#ifdef CONFIG_BME280_EXISTS

static const uint8_t bme280call[] = CONFIG_BME280_MYCALL;

//void fx25_send_packet(uint8_t buf[], int size, int wait, int tnc_mode);
//int get_tnc_mode(void);


/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void BME280_setup()
{
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    readTrim();                    //
}

void BME280_aprs_task(void *arg)
{
    //set Address , Control , PID
    int len;
    int i, j;
    static uint8_t ax25_data[PKT_LEN];
    static uint8_t dst_addr[7] = { 'B' << 1, 'E' << 1, 'A' << 1, 'C' << 1, 'O' << 1, 'N' << 1, SSID };
    static uint8_t src_addr[7] = { 'N' << 1, 'O' << 1, 'C' << 1, 'A' << 1, 'L' << 1, 'L' << 1, SSID | 0x01 };;
    uint8_t *s;
    int seq = 1;
    //int tnc_mode = get_tnc_mode(); // get default TNC mode

    char  lat[] = CONFIG_BME280_LAT;   /*ex.  3539.20N */
    char  lon[] = CONFIG_BME280_LON;   /*ex. 13927.73E */


    // callsign to AX.25 addr
    s = (uint8_t *)ax25_call_to_addr((char *)bme280call);
    if (s) {
	memcpy(src_addr, s, 7);
	src_addr[6] |= 0x01;
    }


    i = 0;
    memcpy(&ax25_data[i], dst_addr, 7); i += 7;
    memcpy(&ax25_data[i], src_addr, 7); i += 7;
    ax25_data[i++] = UI_CONTROL; // Control 0x03, UI frame
    ax25_data[i++] = UI_PID;     // PID 0xf0, no layer 3 protocol


    while (1) {

    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    
    readData();
    
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;

    // for debug 2021.03.05
    printf("TEMP: %.2lfdegC  ", temp_act);
    printf("PRESS: %.2lfhPa  ", press_act);
    printf("HUM: %.2lf%%  ", hum_act);


    i =16; 
    i = i + sprintf((char *)ax25_data+i, "!%s/%s-", lat, lon); //ax25_data[] is not a string
    i = i + sprintf((char *)ax25_data+i, "t:%.2lfdegC ", temp_act);
    i = i + sprintf((char *)ax25_data+i, "p:%.2lfhPa ", press_act);
    i = i + sprintf((char *)ax25_data+i, "h:%.2lf%% ", hum_act);
    i = i + sprintf((char *)ax25_data+i, "  FX.25");

    
    // for debug 2021.03.05
    int k = 0;
    for (k=0; k<i; k++) {
    printf(" %2x", ax25_data[k]);
    }
    printf(" i=%d", i);
    printf(" k=%d\n", k);


    len = i+1;
#if 0
    fx25_send_packet(ax25_data, len, 0, tnc_mode); // 0:do not wait for Queuing, default TNC mode
#else
    {
        tcb_t *tp = &tcb[BME280_APRS_PORT];
        uint8_t *item[2] = { ax25_data, NULL };
        size_t size[2] = { len, 0 };

#ifdef FX25_ENABLE
        fx25_send_packet(tp, item, size, tp->fx25_parity);
#endif
    }
#endif
    vTaskDelay(CONFIG_BME280_INTERVAL * 1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}


void writeReg(uint8_t reg_address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void readData()
{
    uint8_t  data[8] = {0,0,0,0,0,0,0,0};
    uint32_t data_l[8] = {0,0,0,0,0,0,0,0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xf7, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 7, ACK_VAL);
    i2c_master_read_byte(cmd, data+7, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    int i=0;
    for (i=0; i < sizeof(data)/sizeof(uint8_t)-1; i++){
        data_l[i] = data[i];
    }

    pres_raw = (data_l[0] << 12) | (data_l[1] << 4) | (data_l[2] >> 4);
    temp_raw = (data_l[3] << 12) | (data_l[4] << 4) | (data_l[5] >> 4);
    hum_raw  = (data_l[6] << 8) | data_l[7];
}

void readTrim()
{
    uint8_t data[32];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x88, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 23, ACK_VAL);
    i2c_master_read_byte(cmd, data+23, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xa1, ACK_CHECK_EN);
 
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data+24, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xe1, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data+25, 6, ACK_VAL);
    i2c_master_read_byte(cmd, data+31, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30]<< 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}

signed long int calibration_T(signed long int adc_T)
{  
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}



#endif

