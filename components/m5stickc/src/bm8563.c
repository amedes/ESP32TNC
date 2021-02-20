/*
 * driver for RTC BM8563
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"

#include "config.h"
#include "i2c.h"
#include "bm8563.h"

#define BM8563_ADDR 0x51
#define BM8563_REG_CTL1 0x00
#define BM8563_REG_CTL2 0x00
#define BM8563_REG_SEC 0x02
#define BM8563_TIME_LEN 7

static char *TAG = "BM8563";

struct BM8563 bm8563;

void bm8563_write_time(uint8_t *buf)
{
    i2c_write_1byte(BM8563_ADDR, BM8563_REG_CTL1, 0x20); // stop counting

    for (int i = 0; i < BM8563_TIME_LEN; i++) {
	i2c_write_1byte(BM8563_ADDR, BM8563_REG_SEC + i, buf[i]);
    }

    i2c_write_1byte(BM8563_ADDR, BM8563_REG_CTL1, 0x00); // start counting
}

void bm8563_read_time(uint8_t *buf)
{
    i2c_read_bytes(BM8563_ADDR, BM8563_REG_SEC, buf, BM8563_TIME_LEN);
}

void bm8563_init(void)
{
    uint8_t time[BM8563_TIME_LEN];

    i2c_init();

    bm8563_read_time(time);

    if (time[0] & 0x80) {
	ESP_LOGW(TAG, "integrity of the clock information is not guaranteed");

	static const char date_str[] = __DATE__;
	static const char time_str[] = __TIME__;
	static const char mname[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	char mon_str[4];
	int year, month, day, hour, min, sec;

	sscanf(date_str, "%s %d %d", mon_str, &day, &year);
	ESP_LOGI(TAG, "data: %s %d %d\n", mon_str, day, year);
	sscanf(time_str, "%d:%d:%d", &hour, &min, &sec);
	ESP_LOGI(TAG, "time: %d:%d:%d\n", hour, min, sec);
	month = (strstr(mname, mon_str) - mname) / 3 + 1;

	// sec
	time[0] = (sec / 10) * 16 + (sec % 10);

	// min
	time[1] = (min / 10) * 16 + (min % 10);

	// hour
	time[2] = (hour / 10) * 16 + (hour % 10);

	// day
	time[3] = (day / 10) * 16 + (day % 10);

	// week
	struct tm tm = {
	    .tm_sec = sec,
	    .tm_min = min,
	    .tm_hour = hour,
	    .tm_mday = day,
	    .tm_mon = month - 1,
	    .tm_year = year - 1900,
	};

	mktime(&tm); // tm_wyear and tm_wday corrected
	ESP_LOGI(TAG, "week: %s", asctime(&tm));
	time[4] = tm.tm_wday;

	// month
	time[5] = month;

	// year
	year %= 100;
	time[6] = (year / 10) * 16 + (year % 10);

	bm8563_write_time(time); // set local time

    }

    ESP_LOGI(TAG, "time: %02x:%02x:%02x, date: %02x/%02x/%02x, week: %d",
	    time[2], time[1], time[0],
	    time[6], time[5], time[3],
	    time[4]);

#define BCD_VAL(x) ((x >> 4) * 10 + (x & 0x0f))

    struct tm tm = {
	.tm_sec = BCD_VAL(time[0]),
	.tm_min = BCD_VAL(time[1]),
	.tm_hour = BCD_VAL(time[2]),
	.tm_mday = BCD_VAL(time[3]),
	.tm_mon = BCD_VAL(time[5]) - 1,
	.tm_year = 2000 + BCD_VAL(time[6]) - 1900,
	//.tm_gmtoff = 9 * 3600; // assume JST +09:00
    };
	

#ifdef TIME_ZONE
    setenv("TZ", TIME_ZONE, 1);
#else
    setenv("TZ", "JST-9", 1);
#endif
    tzset();

    ESP_LOGI(TAG, "tzname[0] = %s, tzname[1] = %s", tzname[0], tzname[1]);

    time_t gmt = mktime(&tm);

    struct timeval tv = {
	.tv_sec = gmt,
	.tv_usec = 0,
    };

    if (settimeofday(&tv, NULL) != 0) {
	ESP_LOGW(TAG, "settimeofday() fail");
    }

    gettimeofday(&tv, NULL);
    ESP_LOGI(TAG, "ctime(%d) = %s", (int)tv.tv_sec, ctime(&tv.tv_sec));

    // epoch 1970/1/1 00:00:00
    tm.tm_sec = 0;
    tm.tm_min = 0;
    tm.tm_hour = 0;
    tm.tm_mday = 1;
    tm.tm_mon = 0;
    tm.tm_year = 70;

    gmt = mktime(&tm);

    ESP_LOGI(TAG, "GMT offset: %ld sec", gmt);
    
    bm8563.timezone = gmt;

    i2c_write_1byte(BM8563_ADDR, BM8563_REG_CTL2, 0x00); // clear interrupt
}
