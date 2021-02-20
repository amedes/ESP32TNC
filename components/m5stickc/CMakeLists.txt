set(component_srcs
    "src/axp192.c"
    "src/beep.c"
    "src/bm8563.c"
    "src/font.c"
    "src/i2c.c"
    "src/lcd.c"
    "src/led.c"
    "src/m5stickc.c"
    "src/mpu6886.c"
    "src/spi.c"
    "src/tty.c"
    "src/decode_image.c"
    )

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES ""
		       )
