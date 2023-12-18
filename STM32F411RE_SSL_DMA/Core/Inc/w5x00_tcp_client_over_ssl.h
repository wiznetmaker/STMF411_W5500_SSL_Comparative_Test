/*
 * w5x00_tcp_client_over_ssl.h
 *
 *  Created on: 2020. 5. 31.
 *      Author: eziya76@gmail.com
 */

#ifndef _W5X00_TCP_CLIENT_OVER_SSL_H_
#define _W5X00_TCP_CLIENT_OVER_SSL_H_

#include "main.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

void repeating_timer_callback(void);

int wizchip_ssl_init(uint8_t *socket_fd);
int ssl_random_callback(void *p_rng, unsigned char *output, size_t output_len);
int recv_timeout(void *ctx, unsigned char *buf, size_t len, uint32_t timeout);

#endif /* W5X00_TCP_CLIENT_OVER_SSL_H_ */
