/*
______________________________________________________________________________________________________
                                        SPI_IO_Master Library
                                        by: Javier Ruzzante C
                                            (July 2023)
    ______________________________________________________________________________________________________
*/

#ifndef _SPI_IO_Master
#define _SPI_IO_Master

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"

#include "esp_crc.h"

#include "esp_log.h"

// Pins in use
#define GPIO_MOSI 4
#define GPIO_MISO 5
#define GPIO_SCLK 6
#define GPIO_CS 7
#define GPIO_HANDSHAKE 8

#define IO_SPI_HOST SPI2_HOST
#define SPI_USE_POLLING     //Comment out this line for interrupt transactions

extern spi_device_handle_t handle;
extern uint16_t* recvbuf;
extern uint16_t* sendbuf;

extern QueueHandle_t rdySem;

esp_err_t init_spi(void);
void spi_test(void);
esp_err_t spi_write(uint16_t *payload, uint8_t nData);
esp_err_t spi_receive(uint8_t nData);
esp_err_t spi_exchange(uint8_t nData);
uint16_t checksumTable(uint16_t *table, uint8_t nData);



#endif