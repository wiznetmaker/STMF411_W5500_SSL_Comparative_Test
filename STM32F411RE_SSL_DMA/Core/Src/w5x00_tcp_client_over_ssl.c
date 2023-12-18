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

//    mbedtls_ctr_drbg_init(&g_ctr_drbg);
//    mbedtls_ssl_init(&g_ssl);
//    mbedtls_ssl_config_init(&g_conf);

//    if ((retval = mbedtls_ssl_config_defaults(&g_conf,
//                                              MBEDTLS_SSL_IS_CLIENT,
//                                              MBEDTLS_SSL_TRANSPORT_STREAM,
//                                              MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
//    {
//        printf(" failed\n  ! mbedtls_ssl_config_defaults returned %d\n", retval);

//        return -1;
//    }

//    printf(" Socket descriptor %d\n", *socket_fd);

//    mbedtls_ssl_conf_authmode(&g_conf, MBEDTLS_SSL_VERIFY_NONE);
//    mbedtls_ssl_conf_rng(&g_conf, ssl_random_callback, &g_ctr_drbg);
//    mbedtls_ssl_conf_endpoint(&g_conf, MBEDTLS_SSL_IS_CLIENT);
//    mbedtls_ssl_conf_read_timeout(&g_conf, 1000 * 10);

//    if ((retval = mbedtls_ssl_setup(&g_ssl, &g_conf)) != 0)
//    {
//        printf(" failed\n  ! mbedtls_ssl_setup returned %d\n", retval);

//        return -1;
//    }
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
