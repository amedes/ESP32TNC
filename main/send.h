#include "tnc.h"

#define SEND_DEFAULT_PARITY 255

void send_init(tcb_t tcb[]);
void send_bytes(tcb_t *tp, void const *data, size_t size);
void send_packet(tcb_t *tp, void const *data, size_t size, int parity);

