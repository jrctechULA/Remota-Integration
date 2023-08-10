/*
______________________________________________________________________________________________________
                                        SPI_IO_Master Library
                                        by: Javier Ruzzante C
                                            (July 2023)
    ______________________________________________________________________________________________________
*/

#include "SPI_IO_Master.h"
#include "string.h"

esp_err_t init_spi(void) 
{
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    // Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,   //8MHz
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3, // Keep the CS low 0 cycles after transaction
        .queue_size = 5};


    spi_bus_initialize(IO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(IO_SPI_HOST, &devcfg, &handle);

    spi_test();

    return ESP_OK;
}

void spi_test(void){
    esp_err_t res;
    do {
        sendbuf[0] = 18;
        sendbuf[1] = 0;
        sendbuf[2] = 0;
        sendbuf[3] = 0;

        spi_write(sendbuf, 4);
        res = spi_receive(4);
        vTaskDelay(pdMS_TO_TICKS(100));
    } while((res != ESP_OK) || (recvbuf[0] != sendbuf[0]) || (recvbuf[1] != sendbuf[1]) || (recvbuf[2] != sendbuf[2]) || (recvbuf[3] != sendbuf[3]));
    
}

esp_err_t spi_write(uint16_t *payload, uint8_t nData) 
{
    //static const char TAG[] = "SPI Master";

    payload[nData] = checksumTable(payload, nData);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    if (((nData + 1) * 2) % 4)                //Data must be 32bit aligned!
        t.length = (nData + 1) * 16 + 16;
    else
        t.length = (nData + 1) * 16;
    t.tx_buffer = payload;
    t.rx_buffer = NULL;

    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while (xSemaphoreTake(rdySem, portMAX_DELAY) != pdTRUE)
            continue;

    #ifdef SPI_USE_POLLING
        //ESP_LOGW(TAG, "Polling transactions activated!");
        spi_device_polling_transmit(handle, &t);
    #else
       spi_device_transmit(handle, &t);
    #endif
    //ESP_LOGI(TAG, "Transmitted: %u %u %u %u %u", payload[0], payload[1], payload[2], payload[3], payload[4]);
    return ESP_OK;
}

esp_err_t spi_receive(uint8_t nData)
{
    //static const char TAG[] = "SPI Master";
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    if (((nData + 1) * 2) % 4)                //Data must be 32bit aligned!
        t.length = (nData + 1) * 16 + 16;
    else
        t.length = (nData + 1) * 16;
    t.tx_buffer = NULL;
    t.rx_buffer = recvbuf;

    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while (xSemaphoreTake(rdySem, portMAX_DELAY) != pdTRUE)
            continue;

    #ifdef SPI_USE_POLLING
        //ESP_LOGW(TAG, "Polling transactions activated!");
        spi_device_polling_transmit(handle, &t);
    #else
       spi_device_transmit(handle, &t);
    #endif

    uint16_t crcRecv = recvbuf[nData];
    uint16_t crc = checksumTable(recvbuf, nData);
    if (crcRecv != crc){
        ESP_LOGE("CRC Check:", "Communication CRC16 error - Bad checksum!");
        return ESP_FAIL;
    }

    return ESP_OK;
} 

uint16_t checksumTable(uint16_t *table, uint8_t nData){
    //size_t data_len = sizeof(table) / 2;
    uint16_t crc = 0xFFFF; // Valor inicial del CRC-16
    
    for (size_t i = 0; i < nData; i++) {
        uint8_t* bytes = (uint8_t*)&table[i];
        size_t num_bytes = sizeof(table[i]);
        
        for (size_t j = 0; j < num_bytes; j++) {
            //crc = esp_crc16_le(&bytes[j], 1, crc);
            crc = esp_crc16_le(crc, &bytes[j], 1);
        }
    }
    return crc;
}

esp_err_t spi_exchange(uint8_t nData){
    sendbuf[nData] = checksumTable(sendbuf, nData);

    //static const char TAG[] = "SPI Master";
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    if (((nData + 1) * 2) % 4)                //Data must be 32bit aligned!
        t.length = (nData + 1) * 16 + 16;
    else
        t.length = (nData + 1) * 16;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    //xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    while (xSemaphoreTake(rdySem, portMAX_DELAY) != pdTRUE)
            continue;

    #ifdef SPI_USE_POLLING
        //ESP_LOGW(TAG, "Polling transactions activated!");
        spi_device_polling_transmit(handle, &t);
    #else
       spi_device_transmit(handle, &t);
    #endif

    uint16_t crcRecv = recvbuf[nData];
    uint16_t crc = checksumTable(recvbuf, nData);
    if (crcRecv != crc){
        ESP_LOGE("CRC Check:", "Communication CRC16 error - Bad checksum!");
        return ESP_FAIL;
    }
    return ESP_OK;
}