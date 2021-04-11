#pragma once
/*
 * define macro depend on menuconfig choice
 */
#include "sdkconfig.h"

#ifdef CONFIG_M5ATOM
#define M5ATOM 1
#define ENABLE_SOFTMODEM 1
#endif

#ifdef CONFIG_M5ATOM_PORTS
#define M5ATOM_PORTS CONFIG_M5ATOM_PORTS
#endif

#ifdef CONFIG_M5STICKC
#define M5STICKC 1
#define ENABLE_SOFTMODEM 1
#endif

#ifdef CONFIG_FX25TNCR1
#define FX25TNCR1 1
#define ENABLE_TCM3105 1
#define TCM3105_PORT 0
#endif

#ifdef CONFIG_TCM3105_ADC
#define TCM3105_ADC 1
#define ENABLE_SOFTMODEM
#endif

#ifdef CONFIG_FX25TNCR2
#define FX25TNCR2 1
#define ENABLE_SOFTMODEM 1
#endif

#ifdef CONFIG_FX25TNCR3
#define FX25TNCR3 1
#define ENABLE_SOFTMODEM 1
#endif

#ifdef CONFIG_FX25TNCR4
#define FX25TNCR4 1
#define ENABLE_SOFTMODEM 1
#endif

#ifdef CONFIG_ENABLE_TCM3105
#define ENABLE_TCM3105 1
#define TCM3105_PORT 1
#endif

// number of suported ports (supported 2..6 ports)
#ifdef CONFIG_FX25TNCR3_PORTS
#define FX25TNCR3_PORTS CONFIG_FX25TNCR3_PORTS
#endif

#ifdef CONFIG_USE_WIFI
#define USE_WIFI 1
#endif

#ifdef CONFIG_TCP_PORT
#define TCP_PORT CONFIG_TCP_PORT
#endif

#ifdef CONFIG_UDP_PORT
#define UDP_PORT CONFIG_UDP_PORT
#endif

#ifdef CONFIG_TEXT_MODE
#define TEXT_MODE 1
#endif

#ifdef CONFIG_KISS_TXDELAY
#define KISS_TXDELAY CONFIG_KISS_TXDELAY
#endif

#ifdef CONFIG_KISS_P
#define KISS_P CONFIG_KISS_P
#endif

#ifdef CONFIG_KISS_SLOTTIME
#define KISS_SLOTTIME CONFIG_KISS_SLOTTIME
#endif

#ifdef CONFIG_KISS_FULLDUPLEX
#define KISS_FULLDUPLEX CONFIG_KISS_FULLDUPLEX
#endif

#ifdef CONFIG_BEACON
#define BEACON 1
#endif

#ifdef CONFIG_BEACON_CALLSIGN
#define BEACON_CALLSIGN CONFIG_BEACON_CALLSIGN
#endif

#ifdef CONFIG_BEACON_INTERVAL
#define BEACON_INTERVAL CONFIG_BEACON_INTERVAL
#endif

#ifdef CONFIG_BEACON_TEXT
#define BEACON_TEXT CONFIG_BEACON_TEXT
#endif

#ifdef CONFIG_DEBUG
#define DEBUG 1
#endif

#ifdef CONFIG_M5STICKC_AUDIO
#define M5STICKC_AUDIO 1
#endif

#ifdef CONFIG_FX25_ENABLE
#define FX25_ENABLE 1
#endif

#ifdef CONFIG_FX25_PARITY_0
#define FX25_PARITY_0 1
#endif

#ifdef CONFIG_FX25_PARITY_16
#define FX25_PARITY_16 1
#endif

#ifdef CONFIG_FX25_PARITY_32
#define FX25_PARITY_32 1
#endif

#ifdef CONFIG_FX25_PARITY_64
#define FX25_PARITY_64 1
#endif

#ifdef FX25TNCR4
#define GPIO_TXD_PIN 21
#define GPIO_RXD_PIN 19
#define GPIO_PTT_PIN 22
#define GPIO_LED_PIN 2
#define GPIO_CDT_PIN 18
#define GPIO_ENABLE_TCM3105_PIN 5
#endif

#ifdef CONFIG_GPIO_TXD_PIN
#define GPIO_TXD_PIN CONFIG_GPIO_TXD_PIN
#endif

#ifdef CONFIG_GPIO_RXD_PIN
#define GPIO_RXD_PIN CONFIG_GPIO_RXD_PIN
#endif

#ifdef CONFIG_GPIO_PTT_PIN
#define GPIO_PTT_PIN CONFIG_GPIO_PTT_PIN
#endif

#ifdef CONFIG_GPIO_LED_PIN
#define GPIO_LED_PIN CONFIG_GPIO_LED_PIN
#endif

#ifdef CONFIG_GPIO_CDT_PIN
#define GPIO_CDT_PIN CONFIG_GPIO_CDT_PIN
#endif

#ifndef GPIO_TXD_PIN
#define GPIO_TXD_PIN 0
#endif

#ifndef GPIO_RXD_PIN
#define GPIO_RXD_PIN 0
#endif

#ifndef GPIO_PTT_PIN
#define GPIO_PTT_PIN 0
#endif

#ifndef GPIO_LED_PIN
#define GPIO_LED_PIN 0
#endif

#ifndef GPIO_CDT_PIN
#define GPIO_CDT_PIN 0
#endif
