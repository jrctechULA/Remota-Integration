#ifndef REMOTA_GLOBALS_H
#define REMOTA_GLOBALS_H

//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________

#include "nvs.h"
#include "nvs_flash.h"
#include "mbcontroller.h"
#include "ds1307.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________

#define ledYellow 37
#define ledGreen 36
#define pushMasterReset 20

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
#define CFG_MB_MASTER_INTERFACE s3Tables.configTbl[0][13]        //Modbus master interface (TCP or RTU) //40030
#define CFG_MB_MASTER_BAUDRATE_H s3Tables.configTbl[0][14]  //Modbus Master RTU Baudrate High Register  //40031
#define CFG_MB_MASTER_BAUDRATE_L s3Tables.configTbl[0][15]  //Modbus Master RTU Baudrate Low Register   //40032
#define CFG_MB_SLAVE_INTERFACE s3Tables.configTbl[0][16]  //Modbus slave interface (TCP or RTU)         //40033
#define CFG_MB_SLAVE_BAUDRATE_H s3Tables.configTbl[0][17]   //Modbus Slave RTU Baudrate High Register   //40034
#define CFG_MB_SLAVE_BAUDRATE_L s3Tables.configTbl[0][18]   //Modbus Slave RTU Baudrate Low Register    //40035

#define CFG_SLAVE_IP &s3Tables.configTbl[0][19]             //IP address for slave field device (Modbus TCP) 40036 & 40037

#define CFG_GL_TMR_INTERVAL s3Tables.configTbl[0][21]  // Intervalo del timer para el acumulador de volumen diario de gas   40038
#define CFG_GL_FILTER_ALPHA s3Tables.configTbl[0][22]  // Constante de tiempo para el filtro de primer orden aplicado a la medición de flujo  40039

#define CFG_GL_PID_TMR_INTERVAL s3Tables.configTbl[0][23]  // Intervalo del timer para el controlador PID   40040
#define CFG_GL_PID_SP (s3Tables.configTbl[0][41])
#define CFG_GL_PID_KP ((float)s3Tables.configTbl[0][42] / 1000)  //1
#define CFG_GL_PID_CP (s3Tables.configTbl[0][46])               //1
#define CFG_GL_PID_KI ((float)s3Tables.configTbl[0][43] / 1000) //0.08
#define CFG_GL_PID_CI (s3Tables.configTbl[0][47])               //1
#define CFG_GL_PID_KD ((float)s3Tables.configTbl[0][44] / 1000) //0.5
#define CFG_GL_PID_CD (s3Tables.configTbl[0][48])               //1
#define CFG_GL_PID_N  ((float)s3Tables.configTbl[0][45] / 1000) //3

#define CFG_WIFI_MODE (s3Tables.configTbl[0][24])               //2

#define AUX_RTC_YEAR    s3Tables.auxTbl[0][43]
#define AUX_RTC_MONTH   s3Tables.auxTbl[0][44]
#define AUX_RTC_DAY     s3Tables.auxTbl[0][45]
#define AUX_RTC_HOUR    s3Tables.auxTbl[0][46]
#define AUX_RTC_MINUTE  s3Tables.auxTbl[0][47]
#define AUX_RTC_SECOND  s3Tables.auxTbl[0][48]


#define MS24H 86400000                                 // Milisegundos en 24H

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

#define WIFI_NOT_STARTED    0
#define WIFI_CONNECTED      1
#define WIFI_STARTED        2
#define WIFI_GOT_IP         3
#define WIFI_DISCONNECTED   4
#define WIFI_AP_STARTED     5

//_______________________________________________________________________________________________________________
//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________

static const char *TAG = "Remota-Main";
static const char *mbSlaveTAG = "Modbus Slave";
static const char *mbEventChkTAG = "Modbus Event Check";
static const char *wifiTAG = "WiFi Module";


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


//Tasks handles
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xScalingTaskHandle = NULL;
TaskHandle_t xMBEventCheckTaskHandle = NULL;
TaskHandle_t xMBMasterPollTaskHandle = NULL;
TimerHandle_t xGLAcc_Timer = NULL;
TimerHandle_t xGLPID_Timer = NULL;

//The semaphore indicating the slave is ready to receive stuff.
QueueHandle_t rdySem;
QueueHandle_t spiTaskSem;

uint16_t cycleTimeStart, cycleTimeFinish;

//Modbus globals:
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
char slaveIP_str[16] = {'\0'};

// Statically allocate and initialize the spinlock
//static portMUX_TYPE mb_spinlock = portMUX_INITIALIZER_UNLOCKED;

nvs_handle_t app_nvs_handle;

uint8_t modbus_slave_initialized = 0;
uint8_t modbus_master_initialized = 0;
uint8_t modbus_master_connected = 0;
uint8_t mb_master_task_created = 0;
uint8_t resetRequired = 0;

uint32_t msCounter24 = 0;
float last_ftgl = 0;
float ftglFiltered = 0;

float PID_e = 0;
float PID_up = 0;
float PID_ui = 0;
float lastPID_ui = 0;
float PID_ud = 0;

union FloatSplit {
    	float floatValue;
    	struct {
        	uint16_t low;  // Registro bajo
        	uint16_t high; // Registro alto
    	} uint16Values;
};

union FloatSplit PID_u;

//float PID_u = 0;
float lastPID_e = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;

i2c_dev_t dev;

typedef int WiFi_Status_t;
WiFi_Status_t WiFi_Status = WIFI_NOT_STARTED;

char WIFI_SSID[20] = {'\0'};
char WIFI_PASSWORD[20] = {'\0'};
char WIFI_AP_SSID[20] = {'\0'};
char WIFI_AP_PASSWORD[20] = {'\0'};


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

void GLTimerCallBack(TimerHandle_t pxTimer); 
void GLTimerPIDCallBack(TimerHandle_t pxTimer);

esp_err_t init_nvs(void);
esp_err_t read_nvs(char *key, uint16_t *value);
esp_err_t write_nvs(char *key, uint16_t value);
esp_err_t create_table_nvs(char *c, uint8_t tableSize);
esp_err_t create_float_table_nvs(char *c, uint8_t tableSize);
esp_err_t set_configDefaults_nvs(void);

esp_err_t init_FAT_fileSystem(void);
esp_err_t ds1307_init(void);
esp_err_t setTime_ds1307(void);
esp_err_t system_logInput(const char* message);
esp_err_t print_systemLog(void);

esp_err_t Remota_init(void);
void stop_tasks(void);
void resume_tasks(void);
#endif