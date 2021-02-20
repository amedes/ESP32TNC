#include <freertos/ringbuf.h>

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

extern RingbufHandle_t uart_rb;

void uart_init(void);
int uart_add_ringbuf(RingbufHandle_t rb);
int uart_delete_ringbuf(RingbufHandle_t rb);
int uart_add_udp(struct netconn *conn, ip_addr_t *dst, uint16_t port);
int uart_delete_udp(ip_addr_t *dst, uint16_t port);
