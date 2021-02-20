/* SPI Master example: jpeg decoder.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/*
The image used for the effect on the LCD in the SPI master example is stored in flash 
as a jpeg file. This file contains the decode_image routine, which uses the tiny JPEG 
decoder library in ROM to decode this JPEG into a format that can be sent to the display.

Keep in mind that the decoder library cannot handle progressive files (will give 
``Image decoder: jd_prepare failed (8)`` as an error) so make sure to save in the correct
format if you want to use a different image file.
*/

#include <string.h>
#include "decode_image.h"
#include "esp32/rom/tjpgd.h"
#include "esp_log.h"
#include "driver/spi_master.h"

//#include "m5stickc.h"
#include "lcd.h"
#include "spi.h"

//Define the height and width of the jpeg file. Make sure this matches the actual jpeg
//dimensions.
#define TFT_WIDTH  320
#define TFT_HEIGHT 320


const char *TAG="ImageDec";


//Data that is passed from the decoder function to the infunc/outfunc functions.
typedef struct {
    const unsigned char *inData;	//Pointer to jpeg data
    int inPos;						//Current position in jpeg data
    uint8_t ***outData;				//Array of IMAGE_H pointers to arrays of IMAGE_W 16-bit pixel values
    spi_device_handle_t spi;
    uint8_t *buf;			// work area for converted bitmap data
    int bufsize;			// work size for MCU
    uint8_t dbl_buf;			// double buffer pointer
} JpegDev;


//Input function for jpeg decoder. Just returns bytes from the inData field of the JpegDev structure.
static UINT infunc(JDEC *decoder, BYTE *buf, UINT len) 
{
    //Read bytes from input file
    JpegDev *jd=(JpegDev*)decoder->device;
    if (buf!=NULL) memcpy(buf, jd->inData+jd->inPos, len);
    jd->inPos+=len;
    return len;
}

static UINT outfunc(JDEC *decoder, void *bitmap, JRECT *rect) 
{
    JpegDev *jd=(JpegDev*)decoder->device;
    spi_device_handle_t spi = jd->spi;
    spi_transaction_t *tp;

    if (rect->right >= TFT_WIDTH || rect->bottom >= TFT_HEIGHT) return 1;

    // convert from RGB888 to RGB444
    uint8_t *q = &jd->buf[jd->dbl_buf * jd->bufsize];
    uint8_t *p = (uint8_t *)bitmap;
    uint8_t v = 0;

    for (int i = 0; i < 8 * 8 * 3; i++) {
	v <<= 4;
	v |= *p++ >> 4;

	if (i & 1) *q++ = v;
    }

    // issue transaction
    tp = spi_get_trans(spi);
    tp->tx_data[0]=0x2A;           //Column Address Set
    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    tp = spi_get_trans(spi);
    tp->length = 8 * 4;
    tp->user = (void *)1;	// data
    tp->tx_data[0]=rect->left>>8;              //Start Col High
    tp->tx_data[1]=rect->left&0xff;              //Start Col Low
    tp->tx_data[2]=rect->right>>8;       //End Col High
    tp->tx_data[3]=rect->right&0xff;     //End Col Low
    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    tp = spi_get_trans(spi);
    tp->tx_data[0]=0x2B;           //Page address set
    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    tp = spi_get_trans(spi);
    tp->length = 8 * 4;
    tp->user = (void *)1;	// data
    tp->tx_data[0]=rect->top>>8;        //Start page high
    tp->tx_data[1]=rect->top&0xff;      //start page low
    tp->tx_data[2]=rect->bottom>>8;    //end page high
    tp->tx_data[3]=rect->bottom&0xff;  //end page low
    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    tp = spi_get_trans(spi);
    tp->tx_data[0]=0x2C;           //memory write
    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    tp = spi_get_trans(spi);
    tp->user = (void *)1;	// data
    tp->flags=0; //undo SPI_TRANS_USE_TXDATA flag

    tp->tx_buffer = &jd->buf[jd->dbl_buf * jd->bufsize];        //finally send the line data
    jd->dbl_buf = !jd->dbl_buf;
    tp->length = jd->bufsize * 8;          //Data length, in bits

    // wait for previous queued transactions
    //spi_wait_transaction(spi);

    assert(spi_device_queue_trans(spi, tp, portMAX_DELAY) == ESP_OK);

    return 1;
}

//Size of the work space for the jpeg decoder.
#define WORKSZ 3100

//Decode the embedded image into pixel lines that can be used with the rest of the logic.
esp_err_t decode_image(spi_device_handle_t spi, const uint8_t *image_jpg_start) 
{
    char *work=NULL;
    int r;
    JDEC decoder;
    JpegDev jd;
    esp_err_t ret=ESP_OK;

    //Allocate the work space for the jpeg decoder.
    work=calloc(WORKSZ, 1);
    if (work==NULL) {
        ESP_LOGE(TAG, "Cannot allocate workspace");
        ret=ESP_ERR_NO_MEM;
        goto err;
    }

    //Populate fields of the JpegDev struct.
    jd.inData=image_jpg_start;
    jd.inPos=0;
    jd.spi = spi;
    
    //Prepare and decode the jpeg.
    r=jd_prepare(&decoder, infunc, work, WORKSZ, (void*)&jd);
    if (r!=JDR_OK) {
        ESP_LOGE(TAG, "Image decoder: jd_prepare failed (%d)", r);
        ret=ESP_ERR_NOT_SUPPORTED;
        goto err;
    }

    //printf("msx: %d, msy: %d\n", decoder.msx, decoder.msy);
    //printf("width: %d, height: %d\n", decoder.width, decoder.height);
    //printf("%d bytes of work area is used.\n", WORKSZ - decoder.sz_pool);

    jd.bufsize = decoder.msx * 8 * decoder.msy * 8 / 2 * 3;
    jd.buf = calloc(jd.bufsize * 2, 1);
    assert(jd.buf != NULL);
    jd.dbl_buf = 0;

    r=jd_decomp(&decoder, outfunc, 0);
    if (r!=JDR_OK) {
        ESP_LOGE(TAG, "Image decoder: jd_decode failed (%d)", r);
        ret=ESP_ERR_NOT_SUPPORTED;
        goto err;
    }
    
    //All done! Free the work area (as we don't need it anymore) and return victoriously.
    free(jd.buf);
    free(work);
    return ret;
err:
    //Something went wrong! Exit cleanly, de-allocating everything we allocated.
    free(jd.buf);
    free(work);
    return ret;
}
