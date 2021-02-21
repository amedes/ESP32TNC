#pragma once
/*
 * define macro depend on menuconfig choice
 */
#include "sdkconfig.h"

#ifdef CONFIG_M5ATOM
#define M5ATOM 1
#endif

#ifdef CONFIG_M5STICKC
#define M5STICKC 1
#endif

#ifdef CONFIG_FX25TNCR2
#define FX25TNCR2 1
#endif

#ifdef CONFIG_FX25TNCR3
#define FX25TNCR3 1
#endif

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
