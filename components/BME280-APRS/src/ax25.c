/*
    packet related routines
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#define CALLSIGN_LEN 7

char *ax25_call_to_addr(char str[])
{
    static char addr[CALLSIGN_LEN];
    char c, *p = str, *q = addr;
    int ssid = 0;

    memset(addr, ' ' << 1, CALLSIGN_LEN);

    for (int i = 0; i < CALLSIGN_LEN - 1; i++) {
    	c = *p++;

	    if (c == '\0') break;
	    if (c == '-') break;

	    if (!isalnum(c)) break;

	    *q++ = toupper(c) << 1;
    }

    if (c == '-' || *p++ == '-') {
	    ssid = atoi(p);
	    if (ssid < 0 || ssid > 15) ssid = 0;
    }

    addr[CALLSIGN_LEN - 1] = 0x61 | (ssid << 1);

    return addr;
}
