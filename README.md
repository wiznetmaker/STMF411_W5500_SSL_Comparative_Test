# STMF411_W5500_SSL_Comparative_Test
This project is a test that compares the performance when transmitting data received through SPI communication using Dual MCU to an OpenSSL server via W5500 and the performance when transmitting the same amount of data to an OpenSSL server via W5500 using a Single MCU.


#### Dual MCU coremark test 

STM32F411RE(SPI Master) ---> STM32F411RE+W5500(SPI Slave&SSL client) ---> PC(OpenSSL server)

SPI Master Project : https://github.com/aimee0000/STM32F411_W5500_CoreMark_Test/tree/main/STM32F411RE_SPI_Master_DMA

SPI Slave & SSL client Project : https://github.com/aimee0000/STM32F411_W5500_CoreMark_Test/tree/main/STM32F411RE_SPI_Slave_DMA

#### Single MCU coremark test

STM32F411RE+W5500(SSL client) ---> PC(OpenSSL server)

Project : https://github.com/aimee0000/STM32F411_W5500_CoreMark_Test/tree/main/STM32F411RE_SSL_DMA


## Hardware
Device : Nucleo-F411RE, W5500 shield

CPU clock : 100MHz

SPI : use DMA / baudrate 25Mbps / Full duplex 

## Software
CubeMX : https://www.st.com/en/development-tools/stm32cubemx.html

Keil : https://www.keil.com/download/

CoreMark : https://github.com/kagovez/I-CUBE-CoreMark



