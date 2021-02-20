/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

//#include "decode_image.h"
//#include "pretty_effect.h"

#include "font.h"
#include "spi.h"
#include "lcd.h"
#include "axp192.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define M5STICKC_PLUS 1
#define LCD_SPI_NUM SPI2_HOST

#ifdef M5STICKC_PLUS
#define PIN_NUM_MISO -1	// not used
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  13
#define PIN_NUM_CS   5

#define PIN_NUM_DC   23
#define PIN_NUM_RST  18
//#define PIN_NUM_BCKL 0 // not supported

#define LCD_WIDTH (320)
#define LCD_HEIGHT (240)

#define LCD_DISP_TOP 53
#define LCD_DISP_LEFT 40
#define LCD_DISP_WIDTH 240
#define LCD_DISP_HEIGHT 135 

#define LCD_PSL LCD_DISP_LEFT
#define LCD_PEL (LCD_PSL + LCD_DISP_WIDTH - 1)

#define TEXT_DEFAULT_COLOR 0x1f

#else
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5
#endif

#define LCD_TEXT_COLOR 0xfff // RGB444
#define LCD_TEXT_BCOLOR 0x008 // RGB444

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 1

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

static const uint16_t color_palette[16] = {
    0x000,	// 0 - back
    0x00a,	// 1 - blue
    0x0a0,	// 2 - green
    0x0aa,	// 3 - cyan
    0xa00,	// 4 - red
    0xa0a,	// 5 - magenta
    0xa50,	// 6 - brown
    0xaaa,	// 7 - white (light gray)
    0x555,	// 8 - (dark) gray
    0x55f,	// 9 - bright blue
    0x5f5,	// 10 - bright green
    0x5ff,	// 11 - bright cyan
    0xf55,	// 12 - bright red
    0xf5f,	// 13 - bright magent
    0xff5,	// 14 - yellow
    0xfff,	// 15 - bright white
};

struct LCD lcd;

#define HIGH8(x) (x >> 8)
#define LOW8(x) (x & 0xff)

#define TFT_PSL	40
#define TFT_PEL (40+239)

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
static const lcd_init_cmd_t st_init_cmds[]={
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Memory Data Access Control, MX=1, MV=MY=ML=MH=0, RGB=1 */
    {0x36, {(0<<5)|(1<<6)|(1<<3)}, 1},
    /* JLX240 display datasheet */
    {0xb6, {0x0a, 0x82}, 2},
    /* Interface Pixel Format, 12bits/pixel for RGB/MCU interface */
    {0x3A, {0x53}, 1},
    /* Porch Setting */
    //{0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5}, // default value
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    //{0xB7, {0x35}, 1}, // default value
    /* VCOM Setting, VCOM=1.1V */
    {0xBB, {0x28}, 1}, // default value 0x20 (0.9V)
    /* LCM Control, XOR: MX, MH */
    {0xC0, {0x0c}, 1}, // default 0x2c
    /* VDV and VRH Command Enable, enable=1 */
    //{0xC2, {0x01, 0xff}, 2}, // default value
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x10}, 1}, // default value 0x0b
    /* VDV Set, VDV=0 */
    //{0xC4, {0x20}, 1}, // default value
    /* Frame Rate Control2, 60Hz, dot inversion */
    //{0xC6, {0x0f}, 1}, // default
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    //{0xD0, {0xA4, 0xA1}, 1}, // default
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0e, 0x12, 0x14, 0x17}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x31, 0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1b, 0x1e}, 14},
    /* Display Inversion On */
    {0x21, {0}, 0},
    /* Partial Area */
    {0x30, { HIGH8(TFT_PSL), LOW8(TFT_PSL), HIGH8(TFT_PEL), LOW8(TFT_PEL) }, 4},
    /* Partail Display Mode On */
    {0x12, {0}, 0},
    /* Idle Mode On */
    //{0x39, {0}, 0},
    /* Frame Rate Control 1 */
    {0xb3, {0x11, 0x0f, 0x0f}, 3},
    /* Display On */
    {0x29, {0}, 0},

    /* Memory Data Access Control, MX=1, MV=1, MY=ML=MH=0, RGB=0, MH=0 */
    //{0x36, {(0<<7)|(1<<6)|(1<<5)|(0<<4)|(0<<3)|(0<<2)}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    //{0x3A, {0x55}, 1},
    /* Interface Pixel Format, 18bits/pixel for RGB/MCU interface */
    //{0x3A, {0x66}, 1},
    /* Interface Pixel Format, 16M truncated for RGB/MCU interface */
    //{0x3A, {0x67}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    //{0xC0, {0x2C}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    //{0xC6, {0x1f}, 1},
    /* Idle Mode On */
    //{0x39, {0}, 0x80},
    /* Write Content Adaptive Brightness Control and Color Enhancement */
    /* CECTRL=1, CE=3(High), C=2(Still Picture) */
    //{0x55, {0xb3}, 1},
    /* CECTRL=0, CE=0(High), C=2(Still Picture) */
    //{0x55, {0x03}, 1},
    /* CABC Control, LEDONREV=0, DPOFPWM=0, PWMFIX=0, PWMPOL=0 */
    //{0xc7, {0x00}, 1},
    /* Write CABC Minimum Brightness */
    //{0x5e, {0x00}, 1},
    /* Vertical Scrolling Definition, TFA=0, VSA=320, BFA=0 */
    //{0x33, {0x00, 0x00, 0x01, 0x40, 0x00, 0x00}, 6},
    /* Vertical Scroll Start Address of RAM, VSP=160 */
    //{0x37, {0x00, 0xa0}, 2},
    /* Digital Gamma Enable */
    //{0xba, {0x04}, 1},
    /* Gate Control, NL=29(240/8-1), SCN=5(40/8), SM=GS=0 */
    //{0xe4, {0x1d, 0x05, 0x10}, 3},
    /* PWM Frequency Selection */
    //{0xcc, {0x07}, 1},
    /* Partial Area */
    //{0x30, {0x00, 0x28, 0x01, 0x17}, 4},
    //{0x30, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Partial Display Mode On */
    //{0x12, {0}, 0x80},

    /* End of command */
    {0, {0}, 0xff}
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t *tp = spi_get_trans(lcd.spi);
    tp->tx_data[0]=cmd;               //The data is the cmd itself
    ret=spi_device_queue_trans(lcd.spi, tp, portMAX_DELAY);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(const uint8_t *data, int len)
{
    if (len==0) return;             //no need to send anything

    esp_err_t ret;
    spi_transaction_t *tp;

    tp = spi_get_trans(lcd.spi);
    tp->length=len*8;                 //Len is in bytes, transaction length is in bits.
    tp->tx_buffer=data;               //Data
    tp->flags = 0;			// reset USE_TX_DATA flag
    tp->user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_queue_trans(lcd.spi, tp, portMAX_DELAY);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
IRAM_ATTR void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
#if 0
{
    //get_id cmd
    lcd_cmd(spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    return *(uint32_t*)t.rx_data;
}
#else
{
    //get_id cmd
    lcd_cmd(0x04);

    spi_transaction_t *tp = spi_get_trans(spi);

    tp->length=8*3;
    tp->flags = SPI_TRANS_USE_RXDATA;
    tp->user = (void*)1;

    esp_err_t ret = spi_device_queue_trans(spi, tp, portMAX_DELAY);
    assert( ret == ESP_OK );

    // wait for the transaction
    vTaskDelay(100 / portTICK_PERIOD_MS);

    return *(uint32_t*)tp->rx_data;
}
#endif

//Initialize the display
void lcd_init(spi_device_handle_t spi, int lcd_rst_pin)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    lcd.spi = spi; // save spi
    lcd.sleep = false;

    //Initialize non-SPI GPIOs
    gpio_set_direction(lcd_rst_pin, GPIO_MODE_OUTPUT_OD);

    //Reset the display
    gpio_set_level(lcd_rst_pin, 0);
    vTaskDelay(10 / portTICK_RATE_MS);
    gpio_set_level(lcd_rst_pin, 1);
    vTaskDelay(10 / portTICK_RATE_MS);

    // ST7789V2
    lcd_init_cmds = st_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
}

void lcd_madctl(uint8_t param)
{
    lcd_cmd(0x36); // MADCTL
    lcd_data(&param, 1);
}

// expand font pattern with foreground and background color
// buf: buffer for LCD bitmap
// c: character code
// color: color code 12bit RGB444 format, (foreground color << 16) | background color
void font_expand(uint8_t *buf, uint8_t c, uint32_t color)
{
    uint32_t const *fp = (uint32_t const *)font[c];
    uint32_t f;
    uint16_t m;
    uint32_t v;
    uint8_t *bp = buf;
    //uint32_t color = (fcolor << 16) | bcolor;

    if (c == 0) { // fill with background color

	m = color >> 16; // get background color

	uint8_t c0 = m >> 4;
	v = (m << 12) | m;
	uint8_t c1 = v >> 8;
	uint8_t c2 = v;

	for (int i = 0; i < 32; i++) {
	    *bp++ = c0;
	    *bp++ = c1;
	    *bp++ = c2;
	}

	return;
    }

    for (int j = 0; j < 2; j++) {
	f = ~*fp++; // font pattern, get 8x4 dot data, inverted for exchanging foreground and background color

	for (int i = 0; i < 16; i++) {
	    m = color >> ((f & 1)  << 4);
	    f >>= 1;
	
	    *bp++ = m >> 4;
	    v = m << 12;

	    m = color >> ((f & 1)  << 4);
	    f >>= 1;
	    v |= m;

	    *bp++ = v >> 8;
	    *bp++ = v;
	}
    }
}

void font_put(spi_device_handle_t spi, int x, int y, uint8_t c)
{
    if ((x + 8 > LCD_WIDTH) || (y + 8 > LCD_HEIGHT)) return;

    if (c >= 0x80) c = '.';

    static uint8_t buf[2][8 * 8 * 12 / 8];
    static uint8_t dbl_buf = 0;
    uint8_t *bp = buf[dbl_buf];

    // wait for queued transactions
    spi_transaction_t *tp;

    font_expand(bp, c, (color_palette[lcd.color >> 4] << 16) | color_palette[lcd.color & 0x0f]);

    // excute spi transaction

    int xs = y, xe = y + 7;
    int ys = x, ye = x + 7;
 
    // send command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2A;           //Column Address Set

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // send data
    tp = spi_get_trans(spi);
    tp->length = 4 * 8;
    tp->user = (void *)1;

    tp->tx_data[0]= xs >> 8;             //Start Col High
    tp->tx_data[1]= xs & 0xff;          //Start Col Low
    tp->tx_data[2]= xe >> 8;       //End Col High
    tp->tx_data[3]= xe & 0xff;     //End Col Low

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // send command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2B;           //Page address set

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // send data
    tp = spi_get_trans(spi);
    tp->length = 4 * 8;
    tp->user = (void *)1;

    tp->tx_data[0] = ys >> 8;        //Start page high
    tp->tx_data[1] = ys & 0xff;      //start page low
    tp->tx_data[2] = ye >> 8;    //end page high
    tp->tx_data[3] = ye & 0xff;  //end page low

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // send command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2C;           //memory write

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // send data
    tp = spi_get_trans(spi);
    tp->user = (void *)1;

    tp->tx_buffer = buf[dbl_buf];        //finally send the line data
    dbl_buf = !dbl_buf;			// switch double buffer
    tp->length= 8 * 8 * 12;          //Data length, in bits
    tp->flags = 0;

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));
}

void font_puts(spi_device_handle_t spi, int x, int y, uint16_t *data, int len)
{
    if (len > LCD_TEXT_WIDTH) len = LCD_TEXT_WIDTH;

    if (x + len * 8 > LCD_WIDTH || y + 8 > LCD_HEIGHT) return;

    static uint8_t buf[2][8 * 8 * 12 * LCD_TEXT_WIDTH / 8];
    static uint8_t dbl_buf = 0;
    uint8_t *bp = buf[dbl_buf];

    // wait for queued transactions
    spi_transaction_t *tp;

    // make bitmap data from character code
    for (int k = 0; k < len; k++) {
	//font_expand(bp, data[k], (color << 16) | bcolor);
	uint8_t c = data[k];		// code
	uint8_t tc = data[k] >> 8;	// color
	uint32_t color = (color_palette[tc >> 4] << 16) | color_palette[tc & 0x0f];
	font_expand(bp, c, color );
	bp += 8 * 8 * 12 / 8;	// advance buffer pointer
    }

    // excute spi transaction

    int xs = y, xe = y + 7;
    int ys = x, ye = x + len * 8 - 1;

    // command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2A;           //Column Address Set

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // data
    tp = spi_get_trans(spi);
    tp->length = 8 * 4;
    tp->user = (void *)1;

    tp->tx_data[0]= xs >> 8;           //Start Col High
    tp->tx_data[1]= xs & 0xff;         //Start Col Low
    tp->tx_data[2]= xe >> 8;      	//End Col High
    tp->tx_data[3]= xe & 0xff;     	//End Col Low

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2B;           //Page address set

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // data
    tp = spi_get_trans(spi);
    tp->length = 8 * 4;
    tp->user = (void *)1;

    tp->tx_data[0] = ys >> 8;		//Start page high
    tp->tx_data[1] = ys & 0xff;     	//start page low
    tp->tx_data[2] = ye >> 8;    	//end page high
    tp->tx_data[3] = ye & 0xff;  	//end page low

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // command
    tp = spi_get_trans(spi);

    tp->tx_data[0] = 0x2C;           //memory write

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));

    // data
    tp = spi_get_trans(spi);
    tp->user = (void *)1;

    tp->tx_buffer = buf[dbl_buf];        //finally send the line data
    dbl_buf = !dbl_buf;			// switch dobule buffer
    tp->length= len * 8 * 8 * 12;          //Data length, in bits
    tp->flags = 0;

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, tp, portMAX_DELAY));
}

void lcd_text_put(int x, int y, uint8_t c)
{
    lcd.vram[x + y * LCD_TEXT_WIDTH] = (lcd.color << 8) | c;

    if (lcd.sleep) return;

    font_put(lcd.spi, x * 8 + LCD_DISP_LEFT, y * 8 + LCD_DISP_TOP, c);
}

void lcd_redraw(void)
{
    if (lcd.sleep) return;

    for (int y = 0; y < LCD_TEXT_HEIGHT; y++) {
	font_puts(lcd.spi, LCD_DISP_LEFT, y * 8 + LCD_DISP_TOP, &lcd.vram[y * LCD_TEXT_WIDTH], LCD_TEXT_WIDTH);
    }
}

void lcd_scroll(int redraw)
{
    memmove(&lcd.vram[0], &lcd.vram[LCD_TEXT_WIDTH], (LCD_VRAM_SIZE - LCD_TEXT_WIDTH) * sizeof(uint16_t));

    // clear lowest line
    uint16_t *p = &lcd.vram[LCD_VRAM_SIZE - LCD_TEXT_WIDTH];
    for (int i = 0; i < LCD_TEXT_WIDTH; i++) {
	*p++ = lcd.bcolor << 8; // attribute and code 0
    }

    if (redraw) lcd_redraw(); // display bitmap
}

void lcd_clear(void)
{
    for (int i = 0; i < LCD_VRAM_SIZE; i++) {
	lcd.vram[i] = lcd.bcolor << 8;
    }
    lcd_redraw();
    lcd.cursor_x = 0;
    lcd.cursor_y = 0;
}

// carriage return
void lcd_print_cr(void)
{
    lcd.cursor_x = 0;
}

// line feed
void lcd_print_lf(void)
{
    //printf("lcd_print_lf: cur_y = %d, ", lcd.cursor_y);
    lcd.cursor_x = 0;
    if (lcd.cursor_y < LCD_TEXT_HEIGHT - 1) {
	lcd.cursor_y++;
    } else {
	lcd_scroll(true);
    }
    //printf("%d\n", lcd.cursor_y);
}

void lcd_set_color(uint8_t color)
{
    lcd.color = color;
}

void lcd_set_bcolor(uint8_t color)
{
    lcd.bcolor = color;
}

uint8_t lcd_get_color(void)
{
    return lcd.color;
}

#if 0
static void lcd_putchar(uint8_t c)
{
    switch (c) {
	case '\n':
	    lcd_print_lf();
	    return;

	case '\r':
	    lcd_print_cr();
	    return;
    }

    lcd_text_put(lcd.cursor_x, lcd.cursor_y, c);

    if (++lcd.cursor_x >= LCD_TEXT_WIDTH) {

	lcd.cursor_x = 0;

	if (lcd.cursor_y < LCD_TEXT_HEIGHT - 1) {
	    lcd.cursor_y++;
	} else {
	    lcd_scroll(true);
	}
    }
}
#endif

void lcd_text_init(spi_device_handle_t spi)
{
    lcd.spi = spi;
    lcd_set_color(TEXT_DEFAULT_COLOR);
    lcd_set_bcolor(TEXT_DEFAULT_COLOR);
    lcd_clear();
}

void lcd_draw(int cx, int cy, int len)
{
    if (lcd.sleep) return;

    //printf("lcd_draw: cx = %d, cy = %d, len = %d\n", cx, cy, len);
    font_puts(lcd.spi, cx * 8 + LCD_DISP_LEFT, cy * 8 + LCD_DISP_TOP, &lcd.vram[cx + cy * LCD_TEXT_WIDTH], len);
}

void lcd_print(uint8_t const *buf, int n)
{
    int k = n;
    uint8_t const *bp = buf;
    uint16_t *fbp = &lcd.vram[lcd.cursor_x + lcd.cursor_y * LCD_TEXT_WIDTH];
    int xs = lcd.cursor_x; // start position
    int redraw = 0;
    int cnt = 0; // number of characters to draw
    uint8_t c;

    //int tmp = lcd.cursor_y;
    //printf("lcd_print: cursor(%d, %d), ", lcd.cursor_x, lcd.cursor_y);

    while (k-- > 0) {
	c = *bp++; // next character

	switch (c) {
	    case '\n':
		if (redraw) continue;

		if (lcd.cursor_y >= LCD_TEXT_HEIGHT - 1) { // at bottom line
		    lcd_scroll(false);
		    redraw = 1;
		    lcd.cursor_x = 0;
		    xs = 0;
		    
		    continue;
		}

		if (cnt > 0) {
		    lcd_draw(xs, lcd.cursor_y, cnt);
		    cnt = 0;
		}

		lcd.cursor_y++;
		lcd.cursor_x = 0;
		xs = 0;
		fbp = &lcd.vram[lcd.cursor_x + lcd.cursor_y * LCD_TEXT_WIDTH];
		continue;

	    case '\r':
		if (redraw) continue;

		if (cnt > 0) {
		    lcd_draw(xs, lcd.cursor_y, cnt);
		    cnt = 0;
		}

		lcd.cursor_x = 0;
		xs = 0;
		fbp = &lcd.vram[lcd.cursor_x + lcd.cursor_y * LCD_TEXT_WIDTH];
		continue;
	}

	*fbp++ = (lcd.color << 8) | c;
	cnt++;

	if (++lcd.cursor_x >= LCD_TEXT_WIDTH) {

	    lcd.cursor_x = 0;

	    if (lcd.cursor_y >= LCD_TEXT_HEIGHT - 1) { // at bottom

		// need scroll and redraw
		lcd_scroll(false);
		redraw = 1;
		// update frame buffer pointer
		fbp = &lcd.vram[lcd.cursor_x + lcd.cursor_y * LCD_TEXT_WIDTH];

	    } else {

		if (!redraw) {
		    lcd_draw(xs, lcd.cursor_y, cnt);
		}
		lcd.cursor_y++;

	    }

	    xs = 0;
	    cnt = 0;

	}
    }

    if (redraw) {
	lcd_redraw();
    } else if (cnt > 0) {
	lcd_draw(xs, lcd.cursor_y, cnt);
    }

#if 0
    for (int i = 0; i < n; i++) {
	lcd_putchar(buf[i]);
    }
#endif
    //printf("(%d, %d), n = %d\n", lcd.cursor_x, lcd.cursor_y, n);
}

void lcd_write_bytes(void const *buf, int len)
{
    //printf("cursor %d, %d, len = %d\n", lcd.cursor_x, lcd.cursor_y, len);
    lcd_print((uint8_t *)buf, len);
}

void lcd_sleep(uint8_t on)
{
    lcd.sleep = on;

    if (on) {
	lcd_cmd(0x10); // sleep in
	axp192_writeReg(0x12, axp192_readReg(0x12) & 0xfb);
    } else {
	lcd_cmd(0x11); // sleep out
	axp192_writeReg(0x12, axp192_readReg(0x12) | 0x04);
    }
}
