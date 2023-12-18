/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "w5x00_tcp_client_over_ssl.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "mbedtls.h"

#include "mbedtls/x509_crt.h"
#include "mbedtls/error.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */

extern wiz_NetInfo netInfo;

extern mbedtls_ctr_drbg_context ctr_drbg;
extern mbedtls_ssl_config conf;
extern mbedtls_ssl_context ssl;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

static uint32_t millis(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

/* SSL */
int wizchip_ssl_init(uint8_t *socket_fd)
{
    int retval;

		MX_MBEDTLS_Init();
    mbedtls_ssl_set_bio(&ssl, socket_fd, (mbedtls_ssl_send_t *)send, (mbedtls_ssl_recv_t *)recv, recv_timeout);
		
		return retval;
}

static int recv_timeout(void *ctx, unsigned char *buf, size_t len, uint32_t timeout)
{
    uint16_t recv_len = 0;
    uint32_t start_ms = millis();

    do
    {
        getsockopt((uint8_t)(ctx), SO_RECVBUF, &recv_len);

        if (recv_len > 0)
        {
            return recv((uint8_t)ctx, (uint8_t *)buf, (uint16_t)len);
        }
    } while ((millis() - start_ms) < timeout);
		
		g_msec_cnt = 0;

    return MBEDTLS_ERR_SSL_TIMEOUT;
}

/* Timer */
void repeating_timer_callback(void)
{
    g_msec_cnt++;
}

static uint32_t millis(void)
{
    return g_msec_cnt;
}
