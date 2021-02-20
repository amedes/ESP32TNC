#pragma once
/*
 * definition for M5StickC Plus
 */
#include "config.h"

#include "axp192.h"
#include "beep.h"
#include "bm8563.h"
#include "lcd.h"
#include "led.h"
#include "tty.h"
#include "decode_image.h"

#define M5_SYS_SDA 21
#define M5_SYS_SCL 22
#define M5_CLK_SPEED (400 * 1000)

void m5stickc_init(void);
