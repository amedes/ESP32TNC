#include "tnc.h"

#define SEND_PACKET_AX25 0
#define SEND_DEFAULT_PARITY 255
#define CALLSIGN_LEN 7

void send_init(tcb_t tcb[]);
void send_bytes(tcb_t *tp, void const *data, size_t size);
void send_packet(tcb_t *tp, void const *data, size_t size, int parity);
char *make_address(char addr[], char str[]);

