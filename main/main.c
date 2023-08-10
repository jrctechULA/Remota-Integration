//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
//#include "freertos/queue.h"

#include "driver/gpio.h"

//#include "driver/spi_master.h"
#include "SPI_IO_Master.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#include "esp_log.h"              //Already included in "comm_services.c" file

#include "comm_services.c"

#include "mbcontroller.h"

#include "esp_timer.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________

#define ledYellow 37
#define ledGreen 36

#define STACK_SIZE 3072
#define SPI_BUFFER_SIZE 55

#define SPI_TRANSACTION_COUNT_L s3Tables.auxTbl[0][0]
#define SPI_TRANSACTION_COUNT_H s3Tables.auxTbl[0][1]
#define SPI_ERROR_COUNT s3Tables.auxTbl[0][2]

#define SPI_EXCHANGE_TIME s3Tables.auxTbl[0][3]
#define SPI_CYCLE_TIME s3Tables.auxTbl[0][4]

#define CFG_REMOTA_LOG_LEVEL s3Tables.configTbl[0][49]

#define CONFIG_FREERTOS_HZ 100

#define MB_REG_INPUT_START_AREA0    (0)
#define MB_REG_DISCRETE_START_AREA0 (0)
#define MB_REG_COIL_START_AREA0     (0)
#define MB_REG_HOLDING_START_AREA0  (0)
#define MB_REG_HOLDING_START_AREA1  (s3Tables.anSize)   //16
#define MB_REG_HOLDING_START_AREA2  (s3Tables.anSize + s3Tables.configSize)  //66

//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________

DMA_ATTR WORD_ALIGNED_ATTR uint16_t* recvbuf;
DMA_ATTR WORD_ALIGNED_ATTR uint16_t* sendbuf;

spi_device_handle_t handle;

typedef struct {
	uint16_t** anTbl;      // Vector de apuntadores a los vectores analógicos
    uint16_t** digTbl;     // Vector de apuntadores a los vectores Digitales
    uint16_t** configTbl;  // Vector de apuntadores a los vectores de configuración
    uint16_t** auxTbl;     // Vector de apuntadores a los vectores auxiliares
	uint8_t anSize;        // Tamaño de los vectores analógicos
    uint8_t digSize;       // Tamaño de los vectores analógicos
    uint8_t configSize;    // Tamaño de los vectores de configuración
    uint8_t auxSize;       // Tamaño de los vectores auxiliares
	uint8_t numAnTbls;     // Número de vectores analógicos
    uint8_t numDigTbls;    // Número de vectores digitales
    uint8_t numConfigTbls; // Número de vectores de configuración
    uint8_t numAuxTbls;    // Número de vectores auxiliares
} varTables_t;

varTables_t s3Tables;

//The semaphore indicating the slave is ready to receive stuff.
QueueHandle_t rdySem;
QueueHandle_t spiTaskSem;

uint16_t cycleTimeStart, cycleTimeFinish;

//Modbus globals:
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

// Statically allocate and initialize the spinlock
//static portMUX_TYPE mb_spinlock = portMUX_INITIALIZER_UNLOCKED;

static const char *mbSlaveTAG = "Modbus Slave";

//____________________________________________________________________________________________________
// Function prototypes:
//____________________________________________________________________________________________________

esp_err_t tablesInit(varTables_t *tables, 
                     uint8_t numAnTbls,     //Tablas de variables analógicas
                     uint8_t numDigTbls,    //Tablas de variables digitales
                     uint8_t numConfigTbls, //Tablas de configuración
                     uint8_t numAuxTbls,    //Tablas auxiliares
                     uint8_t anSize,        //Tamaño de tablas analógicas
                     uint8_t digSize,       //Tamaño de tablas digitales
                     uint8_t configSize,    //Tamaño de tablas de configuración
                     uint8_t auxSize);      //Tamaño de tablas auxiliares

esp_err_t tablePrint(uint16_t *table, uint8_t size);
esp_err_t tablesUnload(varTables_t *tables);
esp_err_t readAnalogTable(varTables_t *Tables, uint8_t tbl);
esp_err_t readDigitalTable(varTables_t *Tables, uint8_t tbl);
esp_err_t readConfigTable(varTables_t *Tables, uint8_t tbl);
esp_err_t readAuxTable(varTables_t *Tables, uint8_t tbl);
esp_err_t readAllTables(varTables_t *Tables);
esp_err_t readAnalogData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex);
esp_err_t readDigitalData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex);
esp_err_t readConfigData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex);
esp_err_t readAuxData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex);
esp_err_t writeAnalogTable(varTables_t *Tables, uint8_t tbl);
esp_err_t writeDigitalTable(varTables_t *Tables, uint8_t tbl);
esp_err_t writeConfigTable(varTables_t *Tables, uint8_t tbl);
esp_err_t writeAuxTable(varTables_t *Tables, uint8_t tbl);
esp_err_t writeAnalogData(uint8_t tbl, uint8_t dataIndex, uint16_t payload);
esp_err_t writeDigitalData(uint8_t tbl, uint8_t dataIndex, uint16_t payload);
esp_err_t writeConfigData(uint8_t tbl, uint8_t dataIndex, uint16_t payload);
esp_err_t writeAuxData(uint8_t tbl, uint8_t dataIndex, uint16_t payload);

esp_err_t exchangeData(varTables_t *Tables);

void spi_transaction_counter(void);
void print_spi_stats(void);

void spi_task(void *pvParameters);

esp_err_t modbus_slave_init(void);


/*
This ISR is called when the handshake line goes high.
*/
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    uint32_t diff = currtime_us - lasthandshaketime_us;
    if (diff < 50) {
        return; //ignore everything <1ms after an earlier irq
    }
    lasthandshaketime_us = currtime_us;

    //Give the semaphore.
    BaseType_t mustYield = false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) {
        portYIELD_FROM_ISR();
    }
}

//____________________________________________________________________________________________________
// Main program:
//____________________________________________________________________________________________________
void app_main(void)
{
    static const char TAG[] = "Master Main";

    gpio_reset_pin(ledYellow);
    gpio_set_direction(ledYellow, GPIO_MODE_OUTPUT);
    gpio_set_level(ledYellow,0);

    gpio_reset_pin(ledGreen);
    gpio_set_direction(ledGreen, GPIO_MODE_OUTPUT);
    gpio_set_level(ledGreen,0);

    //Set up handshake line interrupt.
    //GPIO config for the handshake line.
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    ESP_LOGI(TAG, "Tamaño del objeto: %i bytes\n", sizeof(s3Tables));  //Imprime el tamaño de la estructura, el cual es constante independientemente del número y tamaño de los vectores
    //tablesInit(&s3Tables, 3,2,10,3);
    tablesInit(&s3Tables, 2,    //Tablas de variables analógicas
                          2,    //Tablas de variables digitales
                          1,    //Tablas de configuración
                          1,    //Tablas auxiliares
                          16,   //Tamaño de tablas analógicas
                          2,    //Tamaño de tablas digitales
                          50,   //Tamaño de tablas de configuración
                          50);  //Tamaño de tablas auxiliares

    // Now the tables have been created, it's time for initial config:
    //i.e.: Set the debug level:
    CFG_REMOTA_LOG_LEVEL = (uint16_t)esp_log_level_get(TAG);

    sendbuf = (uint16_t*)heap_caps_malloc(SPI_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);
    recvbuf = (uint16_t*)heap_caps_malloc(SPI_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);

    //Create the semaphore.
    rdySem = xSemaphoreCreateBinary();
    spiTaskSem = xSemaphoreCreateBinary();
    
    TaskHandle_t xHandle = NULL;
    /* xTaskCreate(spi_task,
                "spi_task",
                STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                &xHandle); */

    xTaskCreatePinnedToCore(spi_task,
                "spi_task",
                STACK_SIZE,
                NULL,
                (UBaseType_t) 1U,       //Priority Level 1
                &xHandle,
                0);          
    xSemaphoreGive(spiTaskSem);

    ethernetInit();

    modbus_slave_init();

    int counter = 0;

    while (1){ 

        //Any change in the Modbus registers should be protected by critical section:

        /* portENTER_CRITICAL(&mb_spinlock);
        for (int i=0; i<s3Tables.anSize; i++){
            s3Tables.anTbl[1][i] +=5;
        }
        for (int i=0; i<s3Tables.digSize; i++){
            s3Tables.digTbl[1][i] +=5;
        }
        portEXIT_CRITICAL(&mb_spinlock); */

        print_spi_stats();
        //esp_log_level_set(TAG, CFG_REMOTA_LOG_LEVEL);
        ESP_LOGD(TAG, "SPI exchange task time: %u us", SPI_EXCHANGE_TIME);
        ESP_LOGD(TAG, "SPI cycle task time: %u us\n", SPI_CYCLE_TIME);
        
        ESP_LOGI(TAG, "Analog inputs table:");
        tablePrint(s3Tables.anTbl[0],  s3Tables.anSize);
        ESP_LOGI(TAG, "Digital inputs table:");
        tablePrint(s3Tables.digTbl[0], s3Tables.digSize);
        ESP_LOGW(TAG, "Analog outputs table:");
        tablePrint(s3Tables.anTbl[1],  s3Tables.anSize);
        ESP_LOGW(TAG, "Digital outputs table:");
        tablePrint(s3Tables.digTbl[1], s3Tables.digSize);

        gpio_set_level(ledGreen, 0);
        vTaskDelay(pdMS_TO_TICKS(500));       
        gpio_set_level(ledGreen, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        counter++;
        if (counter == 15){
            while (xSemaphoreTake(spiTaskSem, portMAX_DELAY) != pdTRUE)
                continue;
            ESP_LOGE(TAG, "SPI taken by app main");
            readAllTables(&s3Tables);
            xSemaphoreGive(spiTaskSem);
            counter = 0;
        }

        //Check for config table changes:
        if (CFG_REMOTA_LOG_LEVEL != (uint16_t)esp_log_level_get(TAG)){
            switch (CFG_REMOTA_LOG_LEVEL){
                case 0:
                    esp_log_level_set(TAG, ESP_LOG_NONE);
                    break;
                case 1:
                    esp_log_level_set(TAG, ESP_LOG_ERROR);
                    break;
                case 2:
                    esp_log_level_set(TAG, ESP_LOG_WARN);
                    break;
                case 3:
                    esp_log_level_set(TAG, ESP_LOG_INFO);
                    break;
                case 4:
                    esp_log_level_set(TAG, ESP_LOG_DEBUG);
                    break;
                case 5:
                    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
                    break;
                default:
                    CFG_REMOTA_LOG_LEVEL = (uint16_t)esp_log_level_get(TAG);
            }
        }
    }
    
    //Liberar memoria
	//tablesUnload(&s3Tables);

}

//____________________________________________________________________________________________________
// Function implementations:
//____________________________________________________________________________________________________

// Tables and dynamic memory related functions:
//____________________________________________________________________________________________________

esp_err_t tablesInit(varTables_t *tables, 
                     uint8_t numAnTbls,
                     uint8_t numDigTbls,
                     uint8_t numConfigTbls,
                     uint8_t numAuxTbls,
                     uint8_t anSize,
                     uint8_t digSize,
                     uint8_t configSize,
                     uint8_t auxSize)
{
    static const char TAG[] = "tablesInit";

    tables->numAnTbls = numAnTbls;
    tables->numDigTbls = numDigTbls;
    tables->numConfigTbls = numConfigTbls;
    tables->numAuxTbls = numAuxTbls;
	tables->anSize = anSize;
    tables->digSize = digSize;
    tables->configSize = configSize;
    tables->auxSize = auxSize;

    tables->anTbl =(uint16_t **)malloc(tables->numAnTbls * sizeof(uint16_t*));
    if (tables->anTbl == NULL){
        ESP_LOGE(TAG, "Error al asignar memoria!\n");
		return ESP_FAIL;
    }

    tables->digTbl =(uint16_t **)malloc(tables->numDigTbls * sizeof(uint16_t*));
    if (tables->digTbl == NULL){
		ESP_LOGE(TAG, "Error al asignar memoria!\n");
		return ESP_FAIL;
	}
    
    tables->configTbl =(uint16_t **)malloc(tables->numConfigTbls * sizeof(uint16_t*));
    if (tables->configTbl == NULL){
		ESP_LOGE(TAG, "Error al asignar memoria!\n");
		return ESP_FAIL;
	}

    tables->auxTbl =(uint16_t **)malloc(tables->numAuxTbls * sizeof(uint16_t*));
    if (tables->auxTbl == NULL){
		ESP_LOGE(TAG, "Error al asignar memoria!\n");
		return ESP_FAIL;
	}

    for (int i=0; i< tables->numAnTbls; i++)
	{
		tables->anTbl[i] = (uint16_t*)malloc(tables->anSize * sizeof(uint16_t));
		if (tables->anTbl[i] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        memset(tables->anTbl[i], 0, tables->anSize * 2);
	}

    for (int i=0; i< tables->numDigTbls; i++)
	{
		tables->digTbl[i] = (uint16_t*)malloc(tables->digSize * sizeof(uint16_t));
		if (tables->digTbl[i] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        memset(tables->digTbl[i], 0, tables->digSize * 2);
	}

    for (int i=0; i< tables->numConfigTbls; i++)
	{
		tables->configTbl[i] = (uint16_t*)malloc(tables->configSize * sizeof(uint16_t));
		if (tables->configTbl[i] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        memset(tables->configTbl[i], 0, tables->configSize * 2);
	}

    for (int i=0; i< tables->numAuxTbls; i++)
	{
		tables->auxTbl[i] = (uint16_t*)malloc(tables->auxSize * sizeof(uint16_t));
		if (tables->auxTbl[i] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        memset(tables->auxTbl[i], 0, tables->auxSize * 2);
	}


    ESP_LOGI(TAG, "Las tablas fueron inicializadas en memoria\n");
    return ESP_OK;
}

esp_err_t tablePrint(uint16_t *table, uint8_t size){
	static const char TAG[] = "Table print";
    esp_log_level_set(TAG, CFG_REMOTA_LOG_LEVEL);
    char text[350] = {'\0'};
    char buffer[10] = {'\0'};
	for (int i=0; i < size; i++){
	    sprintf(buffer, "%u ",table[i]);
        strcat(text, buffer);
        strcat(text, " ");
	}
    ESP_LOGI(TAG, "%s\n", text);
    return ESP_OK;
}

esp_err_t tablesUnload(varTables_t *tables){
    static const char TAG[] = "tablesUnload";
    for (int i=0; i<tables->numAnTbls; i++){  //Libera primero cada vector (int*)
		free(tables->anTbl[i]);
	}
	free(tables->anTbl);                  //Libera el vector de apuntadores (int**)

    for (int i=0; i<tables->numDigTbls; i++){  //Libera primero cada vector (int*)
		free(tables->digTbl[i]);
	}
	free(tables->digTbl);                  //Libera el vector de apuntadores (int**)
    
    for (int i=0; i<tables->numConfigTbls; i++){  //Libera primero cada vector (int*)
		free(tables->configTbl[i]);
	}
	free(tables->configTbl);                  //Libera el vector de apuntadores (int**)

    for (int i=0; i<tables->numAuxTbls; i++){  //Libera primero cada vector (int*)
		free(tables->auxTbl[i]);
	}
	free(tables->auxTbl);                  //Libera el vector de apuntadores (int**)

    ESP_LOGI(TAG, "Memoria liberada");
    return ESP_OK;
}

esp_err_t readAnalogTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    
    sendbuf[0] = 1;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
            
    spi_write(sendbuf, 4);
    
    esp_err_t res = spi_receive(Tables->anSize);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        for (int j=0; j < Tables->anSize; j++){
            Tables->anTbl[tbl][j] = recvbuf[j];
            recvbuf[j]=0;
        }
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t readDigitalTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 2;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);   
    
    esp_err_t res = spi_receive(Tables->digSize);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        for (int j=0; j < Tables->digSize; j++){
            Tables->digTbl[tbl][j] = recvbuf[j];
            recvbuf[j]=0;
        }
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t readConfigTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 9;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);  

    esp_err_t res = spi_receive(Tables->configSize);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        for (int j=0; j < Tables->configSize; j++){
            Tables->configTbl[tbl][j] = recvbuf[j];
            recvbuf[j]=0;
        }
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t readAuxTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 10;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(Tables->auxSize);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        for (int j=0; j < Tables->auxSize; j++){
            Tables->auxTbl[tbl][j] = recvbuf[j];
            recvbuf[j]=0;
        }
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t readAllTables(varTables_t *Tables){
    for (int i=0; i < Tables->numAnTbls; i++){
        esp_err_t r = readAnalogTable(Tables, i);
        if (r == ESP_OK)
            tablePrint(Tables->anTbl[i], Tables->anSize);
    }

    for (int i=0; i < Tables->numDigTbls; i++){
        esp_err_t r = readDigitalTable(Tables, i);
        if (r == ESP_OK)
            tablePrint(Tables->digTbl[i], Tables->digSize);
    }
    return ESP_OK;
}

esp_err_t readAnalogData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex){
    spi_transaction_counter();
    sendbuf[0] = 3;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }
    
    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        Tables->anTbl[tbl][dataIndex] = recvbuf[0];
        recvbuf[0]=0;
        printf("%u\n", (uint16_t)Tables->anTbl[tbl][dataIndex]);
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;

}

esp_err_t readDigitalData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex){
    spi_transaction_counter();
    sendbuf[0] = 4;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        Tables->digTbl[tbl][dataIndex] = recvbuf[0];
        recvbuf[0]=0;
        printf("%u\n", (uint16_t)Tables->digTbl[tbl][dataIndex]);
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t readConfigData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex){
    spi_transaction_counter();
    sendbuf[0] = 15;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);
    
    esp_err_t res = spi_receive(1);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        Tables->configTbl[tbl][dataIndex] = recvbuf[0];
        recvbuf[0]=0;
        printf("%u\n", (uint16_t)Tables->configTbl[tbl][dataIndex]);
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t readAuxData(varTables_t *Tables, uint8_t tbl, uint8_t dataIndex){
    spi_transaction_counter();
    sendbuf[0] = 16;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);
    
    esp_err_t res = spi_receive(1);
    if (res != ESP_OK){             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    if (recvbuf[0] != 0xFFFF){      //Command not recognized error condition check
        Tables->auxTbl[tbl][dataIndex] = recvbuf[0];
        recvbuf[0]=0;
        printf("%u\n", (uint16_t)Tables->auxTbl[tbl][dataIndex]);
    }
    else {
        printf("Communication error! try again...\n");
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeAnalogTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 5;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    for (int i=0; i<s3Tables.anSize; i++)
        sendbuf[i] = s3Tables.anTbl[tbl][i];
    spi_write(sendbuf, s3Tables.anSize);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t writeDigitalTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 6;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    for (int i=0; i<s3Tables.digSize; i++)
        sendbuf[i] = s3Tables.digTbl[tbl][i];
    spi_write(sendbuf, s3Tables.digSize);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK; 
}

esp_err_t writeConfigTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 11;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    for (int i=0; i<s3Tables.configSize; i++)
        sendbuf[i] = s3Tables.configTbl[tbl][i];
    spi_write(sendbuf, s3Tables.configSize);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeAuxTable(varTables_t *Tables, uint8_t tbl){
    spi_transaction_counter();
    sendbuf[0] = 12;
    sendbuf[1] = tbl;
    sendbuf[2] = 0;
    sendbuf[3] = 0;
        
    spi_write(sendbuf, 4);

    for (int i=0; i<s3Tables.auxSize; i++)
        sendbuf[i] = s3Tables.auxTbl[tbl][i];
    spi_write(sendbuf, s3Tables.auxSize);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeAnalogData(uint8_t tbl, uint8_t dataIndex, uint16_t payload){
    spi_transaction_counter();
    sendbuf[0] = 7;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = payload;
       
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeDigitalData(uint8_t tbl, uint8_t dataIndex, uint16_t payload){
    spi_transaction_counter();
    sendbuf[0] = 8;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = payload;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeConfigData(uint8_t tbl, uint8_t dataIndex, uint16_t payload){
    spi_transaction_counter();
    sendbuf[0] = 13;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = payload;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t writeAuxData(uint8_t tbl, uint8_t dataIndex, uint16_t payload){
    spi_transaction_counter();
    sendbuf[0] = 14;
    sendbuf[1] = tbl;
    sendbuf[2] = dataIndex;
    sendbuf[3] = payload;
        
    spi_write(sendbuf, 4);

    esp_err_t res = spi_receive(1);
    if ((res != ESP_OK) || (recvbuf[0] == 0xFFFF)) {             //Checksum error condition check
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t exchangeData(varTables_t *Tables){
    spi_transaction_counter();

    sendbuf[0] = 17;
    sendbuf[1] = 0;
    sendbuf[2] = 0;
    sendbuf[3] = 0;

    spi_write(sendbuf, 4);

    for (int i=0; i<Tables->anSize; i++){
        sendbuf[i] = Tables->anTbl[1][i];
    }
    for (int i=0; i<Tables->digSize; i++){
        sendbuf[Tables->anSize + i] = Tables->digTbl[1][i];
    }

    esp_err_t err = spi_exchange(Tables->anSize + Tables->digSize);
    if (err == ESP_FAIL){
        SPI_ERROR_COUNT++;
        return ESP_FAIL;
    }
        
    //Recover data from recvbuf here:
    for (int i=0; i<Tables->anSize; i++)
        Tables->anTbl[0][i] = recvbuf[i];
    for (int i=0; i<Tables->digSize; i++)
        Tables->digTbl[0][i] = recvbuf[Tables->anSize + i];

    return ESP_OK;
}

void spi_transaction_counter(){
    if (SPI_TRANSACTION_COUNT_L == 0xFFFF){
        SPI_TRANSACTION_COUNT_L = 0;
        if (SPI_TRANSACTION_COUNT_H == 0xFFFF){
            SPI_TRANSACTION_COUNT_H = 0;
            SPI_ERROR_COUNT = 0;
        }
        else
            SPI_TRANSACTION_COUNT_H++;
    }
    else {
       SPI_TRANSACTION_COUNT_L++; 
    }
}

void print_spi_stats(){
    static const char TAG[] = "SPI stats";
    esp_log_level_set(TAG, CFG_REMOTA_LOG_LEVEL);
    uint32_t trans_count = ((uint32_t)(SPI_TRANSACTION_COUNT_H) << 16) | SPI_TRANSACTION_COUNT_L;
    ESP_LOGD(TAG, "Transaction count: %lu Error count: %u Eror ratio: %.2f%%", 
        trans_count, SPI_ERROR_COUNT, (float)SPI_ERROR_COUNT * 100/trans_count);
}

// freeRTOS tasks implementations:
//____________________________________________________________________________________________________
void spi_task(void *pvParameters)
{
    uint16_t exchgTimeStart, exchgTimeFinish;
    
    init_spi();

    xSemaphoreGive(rdySem);
    cycleTimeStart = 0;
    
    while (1)
    {
        cycleTimeFinish = esp_timer_get_time();
        SPI_CYCLE_TIME = cycleTimeFinish - cycleTimeStart + SPI_EXCHANGE_TIME;

        //SPI exchange block:
        //______________________________________________________
        exchgTimeStart = esp_timer_get_time();
        gpio_set_level(ledYellow,1);
        while (xSemaphoreTake(spiTaskSem, portMAX_DELAY) != pdTRUE)
            continue;
        esp_err_t res = exchangeData(&s3Tables);
        if (res != ESP_OK){
            ESP_LOGE("spi_task", "Communication error! Trying to fix...");
            spi_test();
        }
        xSemaphoreGive(spiTaskSem);
        gpio_set_level(ledYellow,0);
        exchgTimeFinish = esp_timer_get_time();
        //______________________________________________________

        cycleTimeStart = esp_timer_get_time();
        SPI_EXCHANGE_TIME = (exchgTimeFinish - exchgTimeStart);

        taskYIELD();
    }
}

esp_err_t modbus_slave_init(void){

    // Stage 1. Modbus Port Initialization:

    void* slave_handler = NULL; // Pointer to allocate interface structure
    // Initialization of Modbus slave for TCP
    esp_err_t err = mbc_slave_init_tcp(&slave_handler);
    if (slave_handler == NULL || err != ESP_OK) {
        // Error handling is performed here
        ESP_LOGE(mbSlaveTAG, "mb controller initialization fail.");
    }

    //Stage 2. Configuring Slave Data Access:

    //Analog Inputs Table:
    reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
    reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
    reg_area.size = (s3Tables.anSize) << 1;                       // Set the size of register storage area in bytes
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Digital Inputs Table:
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_START_AREA0; //0
    reg_area.address = (void*)&s3Tables.digTbl[0][0];
    reg_area.size = 4;
    err = mbc_slave_set_descriptor(reg_area);


    //Analog Outputs Table:
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0; //0
    reg_area.address = (void*)&s3Tables.anTbl[1][0];
    reg_area.size = (s3Tables.anSize) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Digital Outputs Table:
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
    reg_area.address = (void*)&s3Tables.digTbl[1][0];
    reg_area.size = 4;
    err = mbc_slave_set_descriptor(reg_area);

    //Config Table:
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START_AREA1;   //16
    reg_area.address = (void*)&s3Tables.configTbl[0][0];
    reg_area.size = (s3Tables.configSize) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Aux Table:
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START_AREA2;  //66
    reg_area.address = (void*)&s3Tables.auxTbl[0][0];
    reg_area.size = (s3Tables.auxSize) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Stage 3. Slave Communication Options:

    mb_communication_info_t comm_info = {
        .ip_port = 502,                            // Modbus TCP port number (default = 502)
        .ip_addr_type = MB_IPV4,                   // version of IP protocol
        .ip_mode = MB_MODE_TCP,                    // Port communication mode
        .ip_addr = NULL,                           // This field keeps the client IP address to bind, NULL - bind to any client
        .ip_netif_ptr = eth_netif                  // eth_netif - pointer to the corresponding network interface
    };

    // Setup communication parameters and start stack
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    //Stage 4. Slave Communication Start:

    ESP_ERROR_CHECK(mbc_slave_start());

    return ESP_OK;
}