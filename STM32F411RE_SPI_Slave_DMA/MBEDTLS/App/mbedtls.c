/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mbedtls.c
  * Description        : This file provides code for the configuration
  *                      of the mbedtls instances.
  ******************************************************************************
  ******************************************************************************
   * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mbedtls.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Global variables ---------------------------------------------------------*/
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt cert;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_entropy_context entropy;

/* USER CODE BEGIN 2 */
static int ssl_random_callback(void *p_rng, unsigned char *output, size_t output_len)
{
    int i;

    if (output_len <= 0)
    {
        return 1;
    }

    for (i = 0; i < output_len; i++)
    {
        *output++ = rand() % 0xff;
    }

    srand(rand());

    return 0;
}
/* USER CODE END 2 */

/* MBEDTLS init function */
void MX_MBEDTLS_Init(void)
{
   /**
  */
  mbedtls_ssl_init(&ssl);
  mbedtls_ssl_config_init(&conf);
  mbedtls_x509_crt_init(&cert);
  mbedtls_ctr_drbg_init(&ctr_drbg);
  mbedtls_entropy_init( &entropy );
  /* USER CODE BEGIN 3 */
	int retval = 0;
	if ((retval = mbedtls_ssl_config_defaults(&conf,
																						MBEDTLS_SSL_IS_CLIENT,
																						MBEDTLS_SSL_TRANSPORT_STREAM,
																						MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
	{
			printf(" failed\n  ! mbedtls_ssl_config_defaults returned %d\n", retval);
			while(1){};
	}
	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
	mbedtls_ssl_conf_rng(&conf, ssl_random_callback, &ctr_drbg);
	mbedtls_ssl_conf_endpoint(&conf, MBEDTLS_SSL_IS_CLIENT);
	mbedtls_ssl_conf_read_timeout(&conf, 1000 * 10);
	
	
	if ((retval = mbedtls_ssl_setup(&ssl, &conf)) != 0)
	{
			printf(" failed\n  ! mbedtls_ssl_setup returned %d\n", retval);
			while(1){};
	}
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @}
  */

/**
  * @}
  */

