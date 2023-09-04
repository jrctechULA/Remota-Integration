#ifndef REMOTA_GLOBALS_H
#define REMOTA_GLOBALS_H

//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________

#include "nvs.h"
#include "nvs_flash.h"
#include "mbcontroller.h"

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

#define CFG_RUN_PGM s3Tables.configTbl[0][0]              //Run mode or Config mode
#define CFG_OP_MODE s3Tables.configTbl[0][1]              //Operation mode 0-5
#define CFG_IP0 &s3Tables.configTbl[0][2]                 //IP address for Ethernet 0
#define CFG_IP1 &s3Tables.configTbl[0][4]                 //IP address for Ethernet 1
#define CFG_IP2 &s3Tables.configTbl[0][6]                 //IP address for WiFi (Station mode)
#define CFG_IP3 &s3Tables.configTbl[0][8]                 //IP address for WiFi (AP mode)
#define CFG_GW &s3Tables.configTbl[0][10]                 //Gateway
#define CFG_DHCP s3Tables.configTbl[0][12]                //DHCP on/off
#define CFG_MB_MASTER_INTERFACE s3Tables.configTbl[0][13]        //Modbus master interface (TCP or RTU)
#define CFG_MB_MASTER_BAUDRATE s3Tables.configTbl[0][14]  //Modbus RTU Baudrate

#define CONFIG_FREERTOS_HZ 100

#define MB_REG_INPUT_START_AREA0    (0)
#define MB_REG_DISCRETE_START_AREA0 (0)
#define MB_REG_COIL_START_AREA0     (0)
#define MB_REG_HOLDING_START_AREA0  (0)
#define MB_REG_HOLDING_START_AREA1  (s3Tables.anSize)   //16
#define MB_REG_HOLDING_START_AREA2  (s3Tables.anSize + s3Tables.configSize)  //66
#define MB_REG_HOLDING_START_AREA3 (MB_REG_HOLDING_START_AREA2 + 50) //116
#define MB_REG_HOLDING_START_AREA4 (MB_REG_HOLDING_START_AREA3 + 32) //148
#define MB_REG_INPUT_START_AREA1 (MB_REG_INPUT_START_AREA0 + 16) //16

#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

//_______________________________________________________________________________________________________________
//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________

static const char *TAG = "Remota-Main";
static const char *mbSlaveTAG = "Modbus Slave";
static const char *mbEventChkTAG = "Modbus Event Check";

DMA_ATTR WORD_ALIGNED_ATTR uint16_t* recvbuf;
DMA_ATTR WORD_ALIGNED_ATTR uint16_t* sendbuf;

spi_device_handle_t handle;

typedef struct {
	uint16_t** anTbl;      // Vector de apuntadores a los vectores analógicos
    uint16_t** digTbl;     // Vector de apuntadores a los vectores Digitales
    uint16_t** configTbl;  // Vector de apuntadores a los vectores de configuración
    uint16_t** auxTbl;     // Vector de apuntadores a los vectores auxiliares          
    uint16_t** mbTbl16bit;  // Vector de apuntadores a los vectores Modbus de 16 bits   //A reservar según el caso
    uint8_t** mbTbl8bit;   // Vector de apuntadores a los vectores Modbus de 8 bits     //A reservar según el caso
    float** mbTblFloat;     // Vector de apuntadores a los vectores Modbus Float        //A reservar según el caso
    float* scalingFactor; //Vector de factores de escalamiento (pendiente m)
    float* scalingOffset; //Vector de desplazamientos en la escala (corte con y -> b)
    float* scaledValues;  //Vector de apuntadores a los vectores de valores escalados
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

struct natFlow
{
    uint16_t* PTL;
    uint16_t* TTL;
    uint16_t* PTC;
    uint16_t* PTA;
    uint16_t* TTA;
    uint16_t* FT;
};


//Tasks handles
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xScalingTaskHandle = NULL;
TaskHandle_t xMBEventCheckTaskHandle = NULL;
TaskHandle_t xMBMasterPollTaskHandle = NULL;

//The semaphore indicating the slave is ready to receive stuff.
QueueHandle_t rdySem;
QueueHandle_t spiTaskSem;

uint16_t cycleTimeStart, cycleTimeFinish;

//Modbus globals:
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

// Statically allocate and initialize the spinlock
//static portMUX_TYPE mb_spinlock = portMUX_INITIALIZER_UNLOCKED;

nvs_handle_t app_nvs_handle;

uint8_t modbus_slave_initialized = 0;
uint8_t modbus_master_initialized = 0;
uint8_t resetRequired = 0;

// Enumeration of modbus slave addresses accessed by master device
// enum {
//     MB_DEVICE_ADDR1 = 1,
//     MB_SLAVE_COUNT
// };

// Enumeration of all supported CIDs for device
//enum {
//    CID_HOLDING_1 = 0,
//    CID_HOLDING_2,
//    CID_COIL_1,
//    CID_DISCRETE_1
//};

// Modbus Dictionary
//const mb_parameter_descriptor_t device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length,
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
//    { CID_HOLDING_1,                    // CID
//      STR("Holding_1"),                 // Param Name
//      STR("--"),                        // Units
//      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
//      MB_PARAM_HOLDING,                 // Modbus Reg Type
//      0,                                // Reg Start
//      1,                                // Reg Size
//      0,                                // Instance Offset
//      PARAM_TYPE_U16,                   // Data Type
//      2,                                // Data Size
//      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
//      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
//    },

//    { CID_HOLDING_2,                    // CID
//      STR("Holding_2"),                 // Param Name
//      STR("--"),                        // Units
//      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
//      MB_PARAM_HOLDING,                 // Modbus Reg Type
//      1,                                // Reg Start
//      1,                                // Reg Size
//      0,                                // Instance Offset
//      PARAM_TYPE_U16,                   // Data Type
//      2,                                // Data Size
//      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
//      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
//    },

//    { CID_COIL_1,                       // CID
//      STR("Coil_1"),                    // Param Name
//      STR("on/off"),                    // Units
//      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
//      MB_PARAM_COIL,                    // Modbus Reg Type
//      0,                                // Reg Start
//      1,                                // Reg Size
//      0,                                // Instance Offset
//      PARAM_TYPE_U8,                    // Data Type
//      1,                                // Data Size
//      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
//      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
//    },

//    { CID_DISCRETE_1,                    // CID
//      STR("Discrete_1"),                 // Param Name
//      STR("on/off"),                     // Units
//      MB_DEVICE_ADDR1,                   // Modbus Slave Addr
//      MB_PARAM_DISCRETE,                 // Modbus Reg Type
//      0,                                 // Reg Start
//      1,                                 // Reg Size
//      0,                                 // Instance Offset
//      PARAM_TYPE_U8,                     // Data Type
//      1,                                 // Data Size
//      OPTS( 0,0,0 ),                     // Parameter options MIN-MAX-STEP
//      PAR_PERMS_READ_WRITE_TRIGGER       // Access Mode
//    },
//};

// Calculate number of parameters in the table
//uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

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
esp_err_t tablePrintFloat(float *table, uint8_t size);
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
esp_err_t create_modbus_map(void);
esp_err_t modbus_master_init(void);

void scaling_task(void *pvParameters);

void mb_event_check_task(void *pvParameters);

void mb_master_poll_task(void *pvParameters);

esp_err_t init_nvs(void);
esp_err_t read_nvs(char *key, uint16_t *value);
esp_err_t write_nvs(char *key, uint16_t value);
esp_err_t create_table_nvs(char *c, uint8_t tableSize);
esp_err_t create_float_table_nvs(char *c, uint8_t tableSize);
//remota globals block 3 end

#endif