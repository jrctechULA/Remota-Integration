//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

#include "SPI_IO_Master.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#include "esp_log.h"              //Already included in "comm_services.c" file

#include "mbcontroller.h"

#include "esp_timer.h"

#include "remota_globals.h"
#include "remota_var_defs.h"
#include "mb_dictionaries.h"
#include "comm_services.c"

//____________________________________________________________________________________________________
// ISR Functions:
//____________________________________________________________________________________________________

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
    esp_err_t res = init_FAT_fileSystem();
    if ( res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init FAT File System! (%s)", esp_err_to_name(res));
        return;
    }
    else {
        ESP_LOGI(TAG, "FAT File System Initialized (%s)", esp_err_to_name(res));
    }

    res = ds1307_init();
    if ( res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RTC DS1307 (%s)", esp_err_to_name(res));
        return;
    }

    system_logInput("Remota system powered up...");

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

    //nvs_flash_erase();

    ESP_ERROR_CHECK(init_nvs());

    //ESP_ERROR_CHECK(set_configDefaults_nvs());
        
    //Create tables in the nvs namespace (if they don't exist):
    create_table_nvs("C", s3Tables.configSize);     //For config table
    create_table_nvs("A", s3Tables.auxSize);        //For aux table
    create_float_table_nvs("SF", s3Tables.anSize);        //For Scaling factors table
    create_float_table_nvs("SO", s3Tables.anSize);        //For Scaling offsets table

    for (int i = 0; i < s3Tables.configSize; i++)
    {
        char key[5] = {'\0'};
        sprintf(key, "C%i", i);
        read_nvs(key, &s3Tables.configTbl[0][i]);
    }

    for (int i = 0; i < s3Tables.auxSize; i++)
    {
        char key[5] = {'\0'};
        sprintf(key, "A%i", i);
        read_nvs(key, &s3Tables.auxTbl[0][i]);
    }

    for (int i = 0; i < s3Tables.anSize; i++)
    {
        char key[6] = {'\0'};
        sprintf(key, "SF%i", i);
        size_t size = sizeof(float);
        esp_err_t r = nvs_get_blob(app_nvs_handle, key, &s3Tables.scalingFactor[i], &size);
        ESP_ERROR_CHECK(r);
    }

    for (int i = 0; i < s3Tables.anSize; i++)
    {
        char key[6] = {'\0'};
        sprintf(key, "SO%i", i);
        size_t size = sizeof(float);
        esp_err_t r = nvs_get_blob(app_nvs_handle, key, &s3Tables.scalingOffset[i], &size);
        ESP_ERROR_CHECK(r);
    }
    

    sendbuf = (uint16_t*)heap_caps_malloc(SPI_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);
    recvbuf = (uint16_t*)heap_caps_malloc(SPI_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);

    //Create the semaphore.
    rdySem = xSemaphoreCreateBinary();
    spiTaskSem = xSemaphoreCreateBinary();
    
    xTaskCreatePinnedToCore(spi_task,
                "spi_task",
                STACK_SIZE,
                NULL,
                (UBaseType_t) 2U,       //Priority Level 2
                &xSPITaskHandle,
                1);          
    xSemaphoreGive(spiTaskSem);

    xTaskCreatePinnedToCore(scaling_task,
                "scaling_task",
                STACK_SIZE,
                NULL,
                (UBaseType_t) 0U,       //Priority Level 0
                &xScalingTaskHandle,
                0);

    xTaskCreatePinnedToCore(mb_event_check_task,
                "mb_event_check_task",
                STACK_SIZE,
                NULL,
                (UBaseType_t) 2U,       //Priority Level 2
                &xMBEventCheckTaskHandle,
                1);
      
    ethernetInit();

    // WiFi Initialization:
    res = WiFi_init();
    if ( res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi Module (%s)", esp_err_to_name(res));
        return;
    }
    else {
        ESP_LOGI(TAG, "WiFi module succesfully initialized (%s)", esp_err_to_name(res));
    }

    modbus_slave_init();

    if (CFG_OP_MODE == 1) {             //Creates the timer for GAS Volume accumulation, only in Gas Lift OP Mode
        xGLAcc_Timer = xTimerCreate("GL_Timer",
                             pdMS_TO_TICKS(CFG_GL_TMR_INTERVAL),
                             pdTRUE,
                             NULL,
                             GLTimerCallBack);
        xTimerStart(xGLAcc_Timer, pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Gas accumulator timer started with interval %u ms", CFG_GL_TMR_INTERVAL);

        xGLPID_Timer = xTimerCreate("GL_PIDTimer",
                             pdMS_TO_TICKS(CFG_GL_PID_TMR_INTERVAL),
                             pdTRUE,
                             NULL,
                             GLTimerPIDCallBack);
        xTimerStart(xGLPID_Timer, pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Gas PID timer started with interval %u ms", 10);
    }

    // Starts modbus master stack and modbus master poll task only if required
    if ((CFG_OP_MODE == 2) || (CFG_OP_MODE == 3) || (CFG_OP_MODE == 4)){
        //Create and start modbus master poll task:
        xTaskCreatePinnedToCore(mb_master_poll_task,
                "mb_master_poll_task",
                STACK_SIZE,
                NULL,
                (UBaseType_t) 1U,       //Priority Level 1
                &xMBMasterPollTaskHandle,
                1);

        mb_master_task_created = 1;
        
        //Init modbus master stack
        esp_err_t r = modbus_master_init();
        if (r != ESP_OK){
            ESP_LOGE(TAG, "Modbus master initialization error %x", r);
        }
        else {
            modbus_master_connected = 1;
            ESP_LOGI(TAG, "Modbus master initialized");
        }
    }

    

    esp_log_level_set(TAG, CFG_REMOTA_LOG_LEVEL);
    esp_log_level_set(mbSlaveTAG, CFG_REMOTA_LOG_LEVEL);
    esp_log_level_set(mbEventChkTAG, CFG_REMOTA_LOG_LEVEL);


    // Generate a test input for sys_log.log and print its content:
    system_logInput("Remota systems have been succesfully initialized");
    print_systemLog();

    while (1){ 
        if (CFG_RUN_PGM && (resetRequired == 0)){  //Run mode selected:
            //Resume tasks if they're in suspended state:
            if (eTaskGetState(xSPITaskHandle) == eSuspended){
                while (xSemaphoreTake(spiTaskSem, portMAX_DELAY) != pdTRUE)
                        continue;
                vTaskResume(xSPITaskHandle);
                xSemaphoreGive(spiTaskSem);
                ESP_LOGW(TAG, "SPI task resumed...");
            }
            if (eTaskGetState(xScalingTaskHandle) == eSuspended){
                vTaskResume(xScalingTaskHandle);
                ESP_LOGW(TAG, "Scaling task resumed...");
            }
            if (mb_master_task_created){
                if (eTaskGetState(xMBMasterPollTaskHandle) == eSuspended){
                    vTaskResume(xMBMasterPollTaskHandle);
                    ESP_LOGW(TAG, "Modbus master poll task resumed...");
                }
            }
            if (xGLAcc_Timer != NULL){
                if (xTimerIsTimerActive(xGLAcc_Timer) == pdFALSE){
                    xTimerStart(xGLAcc_Timer, 0);
                    ESP_LOGW(TAG, "Gas accumulation timer has been started...");
                }      
            }

            if (xGLPID_Timer != NULL){
                if (xTimerIsTimerActive(xGLPID_Timer) == pdFALSE){
                    xTimerStart(xGLPID_Timer, 0);
                    ESP_LOGW(TAG, "Gas PID timer has been started...");
                    ESP_LOGW(TAG, "Run mode is activated");
                }      
            }
            
            print_spi_stats();
            ESP_LOGD(TAG, "SPI exchange task time: %u us", SPI_EXCHANGE_TIME);
            ESP_LOGD(TAG, "SPI cycle task time: %u us\n", SPI_CYCLE_TIME);

            
            struct tm actualTime;
            ds1307_get_time(&dev, &actualTime);
            char time_str[45];
            //strftime(time_str, sizeof(time_str), "%d-%m-%Y %H:%M:%S", &actualTime);
            strftime(time_str, sizeof(time_str), "%A, %d %B %Y %H:%M:%S", &actualTime);
            ESP_LOGW(TAG, "%s", time_str);
            
            AUX_RTC_YEAR = actualTime.tm_year + 1900;
            AUX_RTC_MONTH = actualTime.tm_mon + 1;
            AUX_RTC_DAY = actualTime.tm_mday;
            AUX_RTC_HOUR = actualTime.tm_hour;
            AUX_RTC_MINUTE = actualTime.tm_min;
            AUX_RTC_SECOND = actualTime.tm_sec;


            switch (CFG_OP_MODE)    //Perform task according to operation mode selected
            {
            case 0:
                /* Pozo de Flujo Natural */
                ESP_LOGI(TAG, "Pozo de Flujo Natural");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");

                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t%u", NF_AI_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t%u", NF_AI_TTL);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t%u", NF_AI_PTC);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t%u", NF_AI_PTA);
                ESP_LOGI(TAG, "Temperatura del casing o anular\t\t(TTA): \t%u", NF_AI_TTA);
                ESP_LOGI(TAG, "Flujo de producción\t\t\t(FT): \t%u\n", NF_AI_FT);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t%.3f", NF_SV_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t%.3f", NF_SV_TTL);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t%.3f", NF_SV_PTC);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t%.3f", NF_SV_PTA);
                ESP_LOGI(TAG, "Temperatura del casing o anular\t\t(TTA): \t%.3f", NF_SV_TTA);
                ESP_LOGI(TAG, "Flujo de producción\t\t\t(FT): \t%.3f\n", NF_SV_FT);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", NF_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", NF_DI_YA2);
                ESP_LOGI(TAG, "Alarma por gas tóxico\t\t\t(GZA): \t\t%u", NF_DI_GZA);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", NF_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", NF_DI_EADC);

                ESP_LOGI(TAG, "Variables digitales de salida leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Apertura/cierre de pozo\t\t\t(XV): \t\t%u\n", NF_DO_XV);

                break;
            case 1:
                /* Pozo de Gas Lift */
                ESP_LOGI(TAG, "Pozo de Gas Lift");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");

                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%u", GL_AI_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%u", GL_AI_TTL);
                ESP_LOGI(TAG, "Posición válvula control flujo al pozo\t(ZT): \t\t%u", GL_AI_ZT);
                ESP_LOGI(TAG, "Presión de gas al pozo\t\t\t(PTGL): \t%u", GL_AI_PTGL);
                ESP_LOGI(TAG, "Temperatura de gas al pozo\t\t(TTGL): \t%u", GL_AI_TTGL);
                ESP_LOGI(TAG, "Flujo de gas al pozo\t\t\t(FTGL): \t%u\n", GL_AI_FTGL);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%.3f", GL_SV_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%.3f", GL_SV_TTL);
                ESP_LOGI(TAG, "Posición válvula control flujo al pozo\t(ZT): \t\t%.3f", GL_SV_ZT);
                ESP_LOGI(TAG, "Presión de gas al pozo\t\t\t(PTGL): \t%.3f", GL_SV_PTGL);
                ESP_LOGI(TAG, "Temperatura de gas al pozo\t\t(TTGL): \t%.3f", GL_SV_TTGL);
                ESP_LOGI(TAG, "Flujo de gas al pozo\t\t\t(FTGL): \t%.3f\n", GL_SV_FTGL);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", GL_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", GL_DI_YA2);
                ESP_LOGI(TAG, "Alarma por gas tóxico\t\t\t(GZA): \t\t%u", GL_DI_GZA);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", GL_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", GL_DI_EADC);

                ESP_LOGI(TAG, "Variables analógicas de salida escritas al módulo de E/S:");
                ESP_LOGI(TAG, "Flujo de gas al pozo\t\t\t(FTGL): \t%.3f\n", PID_u.floatValue);

                ESP_LOGI(TAG, "Volumen diario de gas inyectado:");
                ESP_LOGI(TAG, "Volumen total diario de gas\t\t(FQ): \t\t%.3f\n", GL_FQ24);

                ESP_LOGD(TAG, "GL PID e: %f", PID_e);
                ESP_LOGD(TAG, "GL PID up: %f", PID_up);
                ESP_LOGD(TAG, "GL PID ui: %f", PID_ui);
                ESP_LOGD(TAG, "GL PID ud: %f", PID_ud);
                ESP_LOGD(TAG, "GL PID u: %f\n", PID_u.floatValue); 
                break;
            case 2:
                /* Pozos de Bombeo Mecánico */
                ESP_LOGI(TAG, "Pozos de Bombeo Mecánico");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");

                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t%u", MP_AI_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t%u", MP_AI_TTL);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t%u", MP_AI_PTA);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t%u", MP_AI_PTC);
                ESP_LOGI(TAG, "Golpe por minuto (spm)\t\t\t(SPM): \t%u", MP_AI_SPM);
                ESP_LOGI(TAG, "Ángulo de la viga\t\t\t(ZT): \t%u", MP_AI_ZT);
                ESP_LOGI(TAG, "Carga de la sarta\t\t\t(WT): \t%u\n", MP_AI_WT);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t%.3f", MP_SV_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t%.3f", MP_SV_TTL);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t%.3f", MP_SV_PTA);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t%.3f", MP_SV_PTC);
                ESP_LOGI(TAG, "Golpe por minuto (spm)\t\t\t(SPM): \t%.3f", MP_SV_SPM);
                ESP_LOGI(TAG, "Ángulo de la viga\t\t\t(ZT): \t%.3f", MP_SV_ZT);
                ESP_LOGI(TAG, "Carga de la sarta\t\t\t(WT): \t%.3f\n", MP_SV_WT);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", MP_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", MP_DI_YA2);
                ESP_LOGI(TAG, "Alarma por gas tóxico\t\t\t(GZA): \t\t%u", MP_DI_GZA);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", MP_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", MP_DI_EADC);

                ESP_LOGI(TAG, "Variables digitales de salida leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Encendido/apagado de bombeo\t\t(HS1): \t\t%u\n", MP_DO_HS1);

                if (modbus_master_connected) {
                    ESP_LOGI(TAG, "Variables modbus (input registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Voltaje de motor\t\t(ETM): \t%.3f", MP_IR_ETM);
                    ESP_LOGI(TAG, "Corriente de motor\t(ITM): \t%.3f", MP_IR_ITM);
                    ESP_LOGI(TAG, "Potencia de motor\t(JTM): \t%.3f\n", MP_IR_JTM);

                    ESP_LOGI(TAG, "Variables modbus (holding registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Ajuste de velocidad\t(SCM): \t%.3f\n", MP_HR_SCM);

                    ESP_LOGI(TAG, "Variables modbus (COILS) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Encendido/apagado de bombeo\t(HS1): \t%u\n", MP_COIL_HS1);
                }
                else{
                    ESP_LOGE(TAG, "El equipo esclavo no está conectado... Intentando reconectar...");
                }

                break;
            case 3:
                /* Pozos de bomba electrosumergible */
                ESP_LOGI(TAG, "Pozos de Bomba Electrosumergible");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");

                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%u", ES_AI_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%u", ES_AI_TTL);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t\t%u", ES_AI_PTA);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t\t%u", ES_AI_PTC);
                ESP_LOGI(TAG, "Presión de succión de la bomba\t\t(PTSB): \t%u", ES_AI_PTSB);
                ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t(PTDB): \t%u", ES_AI_PTDB);
                ESP_LOGI(TAG, "Temperatura de succión de la bomba\t(TTSB): \t%u", ES_AI_TTSB);
                ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t(TTDB): \t%u", ES_AI_TTDB);
                ESP_LOGI(TAG, "Vibración de bomba\t\t\t(VT): \t\t%u\n", ES_AI_VT);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%.3f", ES_SV_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%.3f", ES_SV_TTL);
                ESP_LOGI(TAG, "Presión del casing o anular\t\t(PTA): \t\t%.3f", ES_SV_PTA);
                ESP_LOGI(TAG, "Presión de cabezal\t\t\t(PTC): \t\t%.3f", ES_SV_PTC);
                ESP_LOGI(TAG, "Presión de succión de la bomba\t\t(PTSB): \t%.3f", ES_SV_PTSB);
                ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t(PTDB): \t%.3f", ES_SV_PTDB);
                ESP_LOGI(TAG, "Temperatura de succión de la bomba\t(TTSB): \t%.3f", ES_SV_TTSB);
                ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t(TTDB): \t%.3f", ES_SV_TTDB);
                ESP_LOGI(TAG, "Vibración de bomba\t\t\t(VT): \t\t%.3f\n", ES_SV_VT);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", ES_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", ES_DI_YA2);
                ESP_LOGI(TAG, "Alarma por gas tóxico\t\t\t(GZA): \t\t%u", ES_DI_GZA);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", ES_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", ES_DI_EADC);

                ESP_LOGI(TAG, "Variables digitales de salida leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Encendido/apagado de bombeo\t\t(HS1): \t\t%u\n", ES_DO_HS1);

                 if (modbus_master_connected) {
                    ESP_LOGI(TAG, "Variables modbus (input registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Voltaje de motor\t\t\t\t\t(ETM): \t\t%.3f", ES_IR_ETM);
                    ESP_LOGI(TAG, "Corriente de motor\t\t\t\t(ITM): \t\t%.3f", ES_IR_ITM);
                    ESP_LOGI(TAG, "Frecuencia de salida\t\t\t\t(ST1): \t\t%.3f", ES_IR_ST1);
                    ESP_LOGI(TAG, "Temperatura de arrollado de motor\t\t(TTM): \t\t%.3f", ES_IR_TTM);
                    ESP_LOGI(TAG, "Torque de motor\t\t\t\t\t(WTM): \t\t%.3f", ES_IR_WTM);
                    ESP_LOGI(TAG, "Torque de bomba\t\t\t\t\t(WTB): \t\t%.3f", ES_IR_WTB);
                    ESP_LOGI(TAG, "Velocidad de la bomba\t\t\t\t(ST2): \t\t%.3f", ES_IR_ST2);
                    ESP_LOGI(TAG, "Relación de poleas en caja de velocidad (VFD)\t(SFM): \t\t%.3f", ES_IR_SFM);
                    ESP_LOGI(TAG, "Presión de succión de la bomba\t\t\t(PTSB): \t%.3f", ES_IR_PTSB);
                    ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t\t(PTDB): \t%.3f", ES_IR_PTDB);
                    ESP_LOGI(TAG, "Temperatura de succión de la bomba\t\t(TTSB): \t%.3f", ES_IR_TTSB);
                    ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t\t(TTDB): \t%.3f", ES_IR_TTDB);
                    ESP_LOGI(TAG, "Vibración de bomba\t\t\t\t(VT): \t\t%.3f\n", ES_IR_VT);

                    ESP_LOGI(TAG, "Variables de estatus del variador\t\t(YI1): \t\t%u", ES_IR_YI1);
                    ESP_LOGI(TAG, "Fallas actuales del VFD\t\t\t\t(YI2): \t\t%u\n", ES_IR_YI2);

                    ESP_LOGI(TAG, "Variables modbus (holding registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Velocidad de la bomba\t\t(SCM): \t%.3f\n", ES_HR_SCM);

                    ESP_LOGI(TAG, "Variables modbus (DISCRETES) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Encendido/apagado\t\t(YI): \t%u\n", ES_DISCRETE_YI);

                    ESP_LOGI(TAG, "Variables modbus (COILS) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Encendido/apagado\t\t(HS1): \t%u", ES_COIL_HS1);
                    ESP_LOGI(TAG, "Reposición de causa de paro\t(HS2): \t%u\n", ES_COIL_HS2);
                 }
                 else{
                    ESP_LOGE(TAG, "El equipo esclavo no está conectado... Intentando reconectar...");
                }
                break;
            case 4:
                /* Pozos con Bomba de Cavidad Progresiva */
                ESP_LOGI(TAG, "Pozos con Bomba de Cavidad Progresiva");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");

                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%u", PC_AI_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%u", PC_AI_TTL);
                ESP_LOGI(TAG, "Presión de succión de la bomba\t\t(PTSB): \t%u", PC_AI_PTSB);
                ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t(PTDB): \t%u", PC_AI_PTDB);
                ESP_LOGI(TAG, "Temperatura de succión de la bomba\t(TTSB): \t%u", PC_AI_TTSB);
                ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t(TTDB): \t%u", PC_AI_TTDB);
                ESP_LOGI(TAG, "Vibración de bomba\t\t\t(VT): \t\t%u\n", PC_AI_VT);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Presión de la línea de producción\t(PTL): \t\t%.3f", PC_SV_PTL);
                ESP_LOGI(TAG, "Temperatura de la línea de producción\t(TTL): \t\t%.3f", PC_SV_TTL);
                ESP_LOGI(TAG, "Presión de succión de la bomba\t\t(PTSB): \t%.3f", PC_SV_PTSB);
                ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t(PTDB): \t%.3f", PC_SV_PTDB);
                ESP_LOGI(TAG, "Temperatura de succión de la bomba\t(TTSB): \t%.3f", PC_SV_TTSB);
                ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t(TTDB): \t%.3f", PC_SV_TTDB);
                ESP_LOGI(TAG, "Vibración de bomba\t\t\t(VT): \t\t%.3f\n", PC_SV_VT);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", PC_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", PC_DI_YA2);
                ESP_LOGI(TAG, "Alarma por gas tóxico\t\t\t(GZA): \t\t%u", PC_DI_GZA);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", PC_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", PC_DI_EADC);

                ESP_LOGI(TAG, "Variables digitales de salida leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Encendido/apagado de bombeo\t\t(HS1): \t\t%u\n", PC_DO_HS1);

                if (modbus_master_connected) {
                    ESP_LOGI(TAG, "Variables modbus (input registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Torque de motor\t\t\t\t\t(WTM): \t\t%.3f", PC_IR_WTM);
                    ESP_LOGI(TAG, "Torque de bomba\t\t\t\t\t(WTB): \t\t%.3f", PC_IR_WTB);
                    ESP_LOGI(TAG, "Velocidad de la bomba\t\t\t\t(ST2): \t\t%.3f", PC_IR_ST2);
                    ESP_LOGI(TAG, "Relación de poleas en caja de velocidad (VFD)\t(SFM): \t\t%.3f", PC_IR_SFM);
                    ESP_LOGI(TAG, "Presión de succión de la bomba\t\t\t(PTSB): \t%.3f", PC_IR_PTSB);
                    ESP_LOGI(TAG, "Presión de descarga de la bomba\t\t\t(PTDB): \t%.3f", PC_IR_PTDB);
                    ESP_LOGI(TAG, "Temperatura de succión de la bomba\t\t(TTSB): \t%.3f", PC_IR_TTSB);
                    ESP_LOGI(TAG, "Temperatura de descarga de la bomba\t\t(TTDB): \t%.3f", PC_IR_TTDB);
                    ESP_LOGI(TAG, "Vibración de bomba\t\t\t\t(VT): \t\t%.3f\n", PC_IR_VT);

                    ESP_LOGI(TAG, "Variables de estatus del variador\t\t(YI1): \t\t%u", PC_IR_YI1);
                    ESP_LOGI(TAG, "Fallas actuales del VFD\t\t\t\t(YI2): \t\t%u\n", PC_IR_YI2);

                    ESP_LOGI(TAG, "Variables modbus (holding registers) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Velocidad de la bomba\t\t(SCM): \t%.3f\n", PC_HR_SCM);

                    ESP_LOGI(TAG, "Variables modbus (DISCRETES) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Encendido/apagado\t\t(YI): \t%u\n", PC_DISCRETE_YI);

                    ESP_LOGI(TAG, "Variables modbus (COILS) leidas desde el esquipo esclavo:");
                    ESP_LOGI(TAG, "Encendido/apagado\t\t(HS1): \t%u\n", PC_COIL_HS1);
                }
                else{
                    ESP_LOGE(TAG, "El equipo esclavo no está conectado... Intentando reconectar...");
                }
                break;
            case 5:
                /* Estaciones de Válvulas */
                ESP_LOGI(TAG, "Estaciones de Válvulas\n");

                ESP_LOGI(TAG, "Variables analógicas de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Transmisor de presión\t\t\t(PT): \t%u", VS_AI_PT);
                ESP_LOGI(TAG, "Transmisor de temperatura\t\t(TT): \t%u", VS_AI_TT);
                ESP_LOGI(TAG, "Transmisor de interface\t\t\t(AT): \t%u\n", VS_AI_AT);

                ESP_LOGI(TAG, "Variables analógicas de entrada en unidades de ingeniería:");
                ESP_LOGI(TAG, "Transmisor de presión\t\t\t(PT): \t%.3f", VS_SV_PT);
                ESP_LOGI(TAG, "Transmisor de temperatura\t\t(TT): \t%.3f", VS_SV_TT);
                ESP_LOGI(TAG, "Transmisor de interface\t\t\t(AT): \t%.3f\n", VS_SV_AT);

                ESP_LOGI(TAG, "Variables digitales de entrada leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Alarma por falla de sistema\t\t(YA1): \t\t%u", VS_DI_YA1);
                ESP_LOGI(TAG, "Alarma de intruso\t\t\t(YA2): \t\t%u", VS_DI_YA2);
                ESP_LOGI(TAG, "Alarma por falla de voltaje AC\t\t(EAAC): \t%u", VS_DI_EAAC);
                ESP_LOGI(TAG, "Alarma por falla de voltaje DC\t\t(EADC): \t%u\n", VS_DI_EADC);

                ESP_LOGI(TAG, "Detector de herramienta de limpieza\t(XI): \t\t%u", VS_DI_XI);
                ESP_LOGI(TAG, "Bajo nivel de nitrógeno\t\t\t(LNL): \t\t%u", VS_DI_LNL);
                ESP_LOGI(TAG, "Válvula completamente abierta\t\t(OV): \t\t%u", VS_DI_OV);
                ESP_LOGI(TAG, "Válvula completamente cerrada\t\t(CV): \t\t%u\n", VS_DI_CV);

                ESP_LOGI(TAG, "Variables digitales de salida leidas desde el módulo de E/S:");
                ESP_LOGI(TAG, "Cierre rápido de la válvula\t\t(XV): \t\t%u\n", VS_DO_XV);

                break;
            
            default:
                break;
            }

            gpio_set_level(ledGreen, 0);
            vTaskDelay(pdMS_TO_TICKS(500));       
            gpio_set_level(ledGreen, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            
        }
        else{  //Program mode selected
            // Suspend tasks if they're in run state:
            if (eTaskGetState(xSPITaskHandle) != eSuspended){
                while (xSemaphoreTake(spiTaskSem, portMAX_DELAY) != pdTRUE)
                        continue;
                vTaskSuspend(xSPITaskHandle);
                xSemaphoreGive(spiTaskSem);
                ESP_LOGW(TAG, "SPI task has been suspended");
            }
            if (eTaskGetState(xScalingTaskHandle) != eSuspended){
                vTaskSuspend(xScalingTaskHandle);
                ESP_LOGW(TAG, "Scaling task has been suspended");
            }
            if (mb_master_task_created){
                if (eTaskGetState(xMBMasterPollTaskHandle) != eSuspended){
                    vTaskSuspend(xMBMasterPollTaskHandle);
                    ESP_LOGW(TAG, "Modbus master poll task has been suspended");
                }
            }
            if (xGLAcc_Timer != NULL){
                if (xTimerIsTimerActive(xGLAcc_Timer) == pdTRUE){
                    xTimerStop(xGLAcc_Timer, 0);
                    ESP_LOGW(TAG, "Gas accumulation timer has been stopped");
                }
            }

            if (xGLPID_Timer != NULL){
                if (xTimerIsTimerActive(xGLPID_Timer) == pdTRUE){
                    xTimerStop(xGLPID_Timer, 0);
                    ESP_LOGW(TAG, "Gas PID timer has been stopped");
                    ESP_LOGW(TAG, "Program mode is activated. Waiting for setup...");
                }
            }

            //Perform configuration tasks here!
            
        }
        vTaskDelay(pdMS_TO_TICKS(10));  
    }
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
    //static const char TAG[] = "tablesInit";

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

    //Create scaling tables: (The size of these tables are the same as anSize)
    //Scaling factors (m factors in y=mx+b)
    tables->scalingFactor = (float*)malloc(tables->anSize * sizeof(float));
    if (tables->scalingFactor == NULL){
        ESP_LOGE(TAG, "Error al asignar memoria!\n");
        return ESP_FAIL;
    }
    //Default value is 1 (no scaling)
    for (size_t i = 0; i < tables->anSize; i++)
    {
        tables->scalingFactor[i] = 1;
    }
        

    //Scaling offsets (b offset in y=mx+b)
    tables->scalingOffset = (float*)malloc(tables->anSize * sizeof(float));
    if (tables->scalingOffset == NULL){
        ESP_LOGE(TAG, "Error al asignar memoria!\n");
        return ESP_FAIL;
    }
    memset(tables->scalingOffset, 0, tables->anSize * 4);  //Default value is 0 (no offset)

    //Scaled values (Calculated values after scaling factors applied)
    tables->scaledValues = (float*)malloc(tables->anSize * sizeof(float));
    if (tables->scaledValues == NULL){
        ESP_LOGE(TAG, "Error al asignar memoria!\n");
        return ESP_FAIL;
    }
    memset(tables->scaledValues, 0, tables->anSize * 4);

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
    //static const char TAG[] = "SPI stats";
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
    //spi_exchgTime = 0;
    
    while (1)
    {
        cycleTimeFinish = esp_timer_get_time();
        SPI_CYCLE_TIME = (cycleTimeFinish - cycleTimeStart + SPI_EXCHANGE_TIME);

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
    void* slave_handler = NULL; // Pointer to allocate interface structure
    mb_communication_info_t comm_info;
    memset(&comm_info, 0, sizeof(mb_communication_info_t));

    if (CFG_MB_SLAVE_INTERFACE){        // Modbus TCP slave 
        // Stage 1. Modbus Port Initialization:

        // Initialization of Modbus slave for TCP
        esp_err_t err = mbc_slave_init_tcp(&slave_handler);
        if (slave_handler == NULL || err != ESP_OK) {
            // Error handling is performed here
            ESP_LOGE(mbSlaveTAG, "mb controller initialization fail.");
        }

        //Stage 2. Configuring Slave Data Access:

        ESP_ERROR_CHECK(create_modbus_map());

        //Stage 3. Slave Communication Options:

        comm_info.ip_port = 502;
        comm_info.ip_addr_type = MB_IPV4;
        comm_info.ip_mode = MB_MODE_TCP;
        comm_info.ip_addr = NULL;
        comm_info.slave_uid = 1;
        comm_info.ip_netif_ptr = eth_netif;

        // Setup communication parameters and start stack
        ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));
    }
    else {                              // Modbus RTU slave
        // Stage 1. Modbus Port Initialization:

        // Initialization of Modbus slave for RTU
        esp_err_t err = mbc_slave_init(MB_PORT_SERIAL_SLAVE, &slave_handler);
        if (slave_handler == NULL || err != ESP_OK) {
            // Error handling is performed here
            ESP_LOGE(mbSlaveTAG, "mb controller initialization fail.");
        }

        //Stage 2. Configuring Slave Data Access:

        ESP_ERROR_CHECK(create_modbus_map());

        //Stage 3. Slave Communication Options:
        comm_info.mode = MB_MODE_RTU;                    // Communication type
        comm_info.slave_addr = 1;                        // Short address of the slave
        comm_info.port = 1;                              // UART physical port number
        comm_info.baudrate = ((uint32_t)CFG_MB_SLAVE_BAUDRATE_H << 16) | CFG_MB_SLAVE_BAUDRATE_L;         // Baud rate for communication
        comm_info.parity = MB_PARITY_NONE;                // Parity option

        // Setup communication parameters and start stack
        ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

        // Set UART pin numbers
        ESP_ERROR_CHECK(uart_set_pin(1, 15, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_LOGI(TAG, "Modbus slave RTU initialized at: %lu bps", comm_info.baudrate);
    }

    //Stage 4. Slave Communication Start:

    ESP_ERROR_CHECK(mbc_slave_start());

    modbus_slave_initialized = 1;

    return ESP_OK;
}

esp_err_t create_modbus_map(void){
    //____________________________________________________________________________________________________
    //Basic Modbus map:
    //____________________________________________________________________________________________________
    //Config Table:
    reg_area.type = MB_PARAM_HOLDING;                               // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA1;   //16      // Offset of register area in Modbus protocol
    reg_area.address = (void*)&s3Tables.configTbl[0][0];            // Set pointer to storage instance
    reg_area.size = (s3Tables.configSize) << 1;                     // Set the size of register storage area in bytes
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Aux Table:
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START_AREA2;  //66
    reg_area.address = (void*)&s3Tables.auxTbl[0][0];
    reg_area.size = (s3Tables.auxSize) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Digital Inputs Table:
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = MB_REG_DISCRETE_START_AREA0; //0
    reg_area.address = (void*)&s3Tables.digTbl[0][0];
    reg_area.size = 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //____________________________________________________________________________________________________
    // Extended Modbus map: (Depending of CFG_OP_MODE)
    //____________________________________________________________________________________________________
    switch (CFG_OP_MODE)
    {
    case 0:      /* Pozo de Flujo Natural */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 6 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Outputs Table:
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
        reg_area.address = (void*)&s3Tables.digTbl[1][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        
        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = MB_REG_INPUT_START_AREA1;  //16
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        break;

    case 1:     /* Pozo de Gas Lift */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 6 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Analog Outputs Table:
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA0; //0
        reg_area.address = (void*)&s3Tables.anTbl[1][0];
        reg_area.size = 2 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = MB_REG_INPUT_START_AREA1;  //16
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 6 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTblFloat =(float **)malloc(1 * sizeof(float*));
        if (s3Tables.mbTblFloat == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTblFloat[0] = (float*)malloc(4 * sizeof(float));
		if (s3Tables.mbTblFloat[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        for (int i = 0; i < 4; i++)
            s3Tables.mbTblFloat[0][i] = 0;

        // Gas accumulation input registers (24H, 48H, 72H)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 28;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][0];
        reg_area.size = 4 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Nota: El volumen total diario de gas se calculará en la tabla mbTblFloat, en los registros 0,1 y 2.
        break;

        
    case 2:     /* Pozos de Bombeo Mecánico */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 7 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Outputs Table:
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
        reg_area.address = (void*)&s3Tables.digTbl[1][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = MB_REG_INPUT_START_AREA1;  //16
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        
        s3Tables.mbTblFloat =(float **)malloc(1 * sizeof(float*));
        if (s3Tables.mbTblFloat == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTblFloat[0] = (float*)malloc(4 * sizeof(float));
		if (s3Tables.mbTblFloat[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        for (int i = 0; i < 4; i++)
            s3Tables.mbTblFloat[0][i] = 0;

        // Modbus input registers (for the slave device)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 30;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][0];
        reg_area.size = 3 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        // Modbus holding registers (for the slave device)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = 180;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][3];
        reg_area.size = 1 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTbl8bit =(uint8_t **)malloc(1 * sizeof(uint8_t*));
        if (s3Tables.mbTbl8bit == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTbl8bit[0] = (uint8_t*)malloc(1 * sizeof(uint8_t));
		if (s3Tables.mbTbl8bit[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        s3Tables.mbTbl8bit[0][0] = 0;

        // Modbus coil registers (for the slave device)
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.mbTbl8bit[0][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        break;

    case 3:     /* Pozos de bomba electrosumergible */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 9 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Outputs Table:
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
        reg_area.address = (void*)&s3Tables.digTbl[1][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 9 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 9 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = MB_REG_INPUT_START_AREA1;  //16
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 9 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTblFloat =(float **)malloc(1 * sizeof(float*));
        if (s3Tables.mbTblFloat == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTblFloat[0] = (float*)malloc(14 * sizeof(float));
		if (s3Tables.mbTblFloat[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        for (int i = 0; i < 14; i++)
            s3Tables.mbTblFloat[0][i] = 0;

        // Modbus input registers (for the slave device)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 34;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][0];
        reg_area.size = 13 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        // Modbus holding registers (for the slave device)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = 180;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][13];
        reg_area.size = 1 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTbl16bit =(uint16_t **)malloc(1 * sizeof(uint16_t*));
        if (s3Tables.mbTbl16bit == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTbl16bit[0] = (uint16_t*)malloc(2 * sizeof(uint16_t));
		if (s3Tables.mbTbl16bit[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        s3Tables.mbTbl16bit[0][0] = 0;
        s3Tables.mbTbl16bit[0][1] = 0;
        
        
        // Modbus input registers (for the slave device)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 60;
        reg_area.address = (void*)&s3Tables.mbTbl16bit[0][0];
        reg_area.size = 2 << 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        
        s3Tables.mbTbl8bit =(uint8_t **)malloc(1 * sizeof(uint8_t*));
        if (s3Tables.mbTbl8bit == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTbl8bit[0] = (uint8_t*)malloc(2 * sizeof(uint8_t));
		if (s3Tables.mbTbl8bit[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        s3Tables.mbTbl8bit[0][0] = 0;
        s3Tables.mbTbl8bit[0][1] = 0;

        //Modbus discrete inputs (for the slave device):
        reg_area.type = MB_PARAM_DISCRETE;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.mbTbl8bit[0][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Modbus coils (for the slave device):
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.mbTbl8bit[0][1];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        
        break;

    case 4:     /* Pozos con Bomba de Cavidad Progresiva */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 7 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Outputs Table:
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
        reg_area.address = (void*)&s3Tables.digTbl[1][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = MB_REG_INPUT_START_AREA1;  //16
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 7 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTblFloat =(float **)malloc(1 * sizeof(float*));
        if (s3Tables.mbTblFloat == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTblFloat[0] = (float*)malloc(10 * sizeof(float));
		if (s3Tables.mbTblFloat[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        for (int i = 0; i < 10; i++)
            s3Tables.mbTblFloat[0][i] = 0;

        // Modbus input registers (for the slave device)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 30;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][0];
        reg_area.size = 9 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        // Modbus holding registers (for the slave device)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = 180;
        reg_area.address = (void*)&s3Tables.mbTblFloat[0][9];
        reg_area.size = 1 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));


        s3Tables.mbTbl16bit =(uint16_t **)malloc(1 * sizeof(uint16_t*));
        if (s3Tables.mbTbl16bit == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTbl16bit[0] = (uint16_t*)malloc(2 * sizeof(uint16_t));
		if (s3Tables.mbTbl16bit[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        
        s3Tables.mbTbl16bit[0][0] = 0;
        s3Tables.mbTbl16bit[0][1] = 0;

        // Modbus input registers (for the slave device)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 48;
        reg_area.address = (void*)&s3Tables.mbTbl16bit[0][0];
        reg_area.size = 2 << 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        s3Tables.mbTbl8bit =(uint8_t **)malloc(1 * sizeof(uint8_t*));
        if (s3Tables.mbTbl8bit == NULL){
            ESP_LOGE(TAG, "Error al asignar memoria!\n");
		    return ESP_FAIL;
        }
        s3Tables.mbTbl8bit[0] = (uint8_t*)malloc(2 * sizeof(uint8_t));
		if (s3Tables.mbTbl8bit[0] == NULL){
			ESP_LOGE(TAG, "Error al asignar memoria!\n");
			return ESP_FAIL;
		}
        s3Tables.mbTbl8bit[0][0] = 0;
        s3Tables.mbTbl8bit[0][1] = 0;
        
        //Modbus discrete inputs (for the slave device):
        reg_area.type = MB_PARAM_DISCRETE;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.mbTbl8bit[0][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Modbus coils (for the slave device):
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.mbTbl8bit[0][1];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        

        break;
    case 5:     /* Estaciones de Válvulas */
        //Analog Inputs Table:
        reg_area.type = MB_PARAM_INPUT;                               // Set type of register area
        reg_area.start_offset = MB_REG_INPUT_START_AREA0;  //0        // Offset of register area in Modbus protocol
        reg_area.address = (void*)&s3Tables.anTbl[0][0];              // Set pointer to storage instance
        reg_area.size = 3 << 1;                                       // Set the size of register storage area in bytes
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Outputs Table:
        reg_area.type = MB_PARAM_COIL;
        reg_area.start_offset = MB_REG_COIL_START_AREA0;  //0
        reg_area.address = (void*)&s3Tables.digTbl[1][0];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Digital Inputs Table: (Aditional)
        reg_area.type = MB_PARAM_DISCRETE;
        reg_area.start_offset = 8;
        reg_area.address = (void*)&s3Tables.digTbl[0][1];
        reg_area.size = 1;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Factors Table: (Slopes: m)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA3;  //116
        reg_area.address = (void*)&s3Tables.scalingFactor[0];
        reg_area.size = 3 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Offsets Table: (y cuts: b)
        reg_area.type = MB_PARAM_HOLDING;
        reg_area.start_offset = MB_REG_HOLDING_START_AREA4;  //148
        reg_area.address = (void*)&s3Tables.scalingOffset[0];
        reg_area.size = 3 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

        //Scaling Values Table: (Analog inputs after scaling is applied)
        reg_area.type = MB_PARAM_INPUT;
        reg_area.start_offset = 3;
        reg_area.address = (void*)&s3Tables.scaledValues[0];
        reg_area.size = 3 * 4;
        ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));
        break;


    
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t modbus_master_init(void){

    esp_err_t err;
    mb_communication_info_t comm_info;
    memset(&comm_info, 0, sizeof(mb_communication_info_t));

    if (CFG_MB_MASTER_INTERFACE){
        void* master_handler = NULL; // Pointer to allocate interface structure
        // Initialization of Modbus master for TCP/IP
        err = mbc_master_init_tcp(&master_handler);
        if (master_handler == NULL || err != ESP_OK) {
            ESP_LOGE(TAG, "mb controller initialization fail.");
        }

        //char slaveIP_str[16] = {'\0'};    //Moved to remota_globals.h
        uint8_t IP0[4] = {0};
        IP0[0] = *CFG_SLAVE_IP >> 8;
        IP0[1] = *CFG_SLAVE_IP & 0x00FF;
        IP0[2] = *(CFG_SLAVE_IP+1) >> 8;
        IP0[3] = *(CFG_SLAVE_IP+1) & 0x00FF;
        sprintf(slaveIP_str, "%hhu.%hhu.%hhu.%hhu", IP0[0], IP0[1], IP0[2], IP0[3]);
        
        ESP_LOGV(TAG, "Slave IP Address: %s", slaveIP_str);

        const char* slave_ip_address_table[2] = {
            slaveIP_str,       // Address corresponds to UID1 and set to predefined value by user
            NULL               // end of table
        };

        /* const char* slave_ip_address_table[2] = {
            "172.16.0.4",     // Address corresponds to UID1 and set to predefined value by user
            NULL               // end of table
        }; */

        comm_info.ip_port = 502;                    // Modbus TCP port number (default = 502)
        comm_info.ip_addr_type = MB_IPV4;                   // version of IP protocol
        comm_info.ip_mode = MB_MODE_TCP;                    // Port communication mode
        comm_info.ip_addr = (void*)slave_ip_address_table;  // assign table of IP addresses
        comm_info.ip_netif_ptr = eth_netif;              // esp_netif_ptr pointer to the corresponding network interface

        ESP_ERROR_CHECK(mbc_master_setup((void*)&comm_info));
        
    }
    else {
        void* master_handler = NULL; // Pointer to allocate interface structure
        // Initialization of Modbus master for serial port
        err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
        if (master_handler == NULL || err != ESP_OK) {
            ESP_LOGE(TAG, "mb controller initialization fail.");
        }

        comm_info.port = 2;                  // Serial port number
        comm_info.mode = MB_MODE_RTU;        // Modbus mode of communication (MB_MODE_RTU or MB_MODE_ASCII)
        comm_info.baudrate = ((uint32_t)CFG_MB_MASTER_BAUDRATE_H << 16) | CFG_MB_MASTER_BAUDRATE_L;           // Modbus communication baud rate
        comm_info.parity = MB_PARITY_NONE;    // parity option for serial port

        ESP_ERROR_CHECK(mbc_master_setup((void*)&comm_info));

        // Set UART pin numbers
        ESP_ERROR_CHECK(uart_set_pin(2, 41, 42, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_LOGI(TAG, "Modbus master RTU initialized at: %lu bps", comm_info.baudrate);
    }

    switch (CFG_OP_MODE)    //Select appropiate dictionary depending on OP Mode
    {
    case 2: //Mechanical Pump Wells
        ESP_ERROR_CHECK(mbc_master_set_descriptor(&MP_device_parameters[0], num_MP_device_parameters));
        break;
    case 3: //Electro Submersible Pump Wells
        ESP_ERROR_CHECK(mbc_master_set_descriptor(&ES_device_parameters[0], num_ES_device_parameters));
        break;
    case 4: //Progressive Cavity Pump Wells
        ESP_ERROR_CHECK(mbc_master_set_descriptor(&PC_device_parameters[0], num_PC_device_parameters));
        break;
    }
    

    err = mbc_master_start();
    modbus_master_initialized = 1;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mb controller start fail, err=%x.", err);
    }
    return err;
}

void scaling_task(void *pvParameters){
    float y, m, x, b;

    while (1)
    {
        for (int i = 0; i < s3Tables.anSize; i++){
            m = s3Tables.scalingFactor[i];
            x = s3Tables.anTbl[0][i];
            b = s3Tables.scalingOffset[i];
            y =  m * x + b;
            s3Tables.scaledValues[i] = y;
        }
        taskYIELD();
    }
    
}

void mb_event_check_task(void *pvParameters){
    mb_param_info_t reg_info;
    size_t size = sizeof(float);

    while(!modbus_slave_initialized){
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
    }

    while (1)
    {
        memset(&reg_info, 0, sizeof(mb_param_info_t));
        mb_event_group_t event = mbc_slave_check_event(MB_EVENT_HOLDING_REG_WR);
        
        //if (event & MB_EVENT_HOLDING_REG_WR){  
            for (int i = 0; i<=CONFIG_FMB_CONTROLLER_NOTIFY_QUEUE_SIZE; i++)
            {
                mbc_slave_get_param_info(&reg_info, 10 / portTICK_PERIOD_MS);
                ESP_LOGV(mbEventChkTAG, "HOLDING (%lu us), ADDR:%lu, TYPE:%lu, SIZE:%u",
                        (uint32_t)reg_info.time_stamp,
                        (uint32_t)reg_info.mb_offset,
                        (uint32_t)reg_info.type,
                        reg_info.size);

                if(reg_info.type == MB_EVENT_HOLDING_REG_WR){
                    if((reg_info.mb_offset < 16)){
                        ESP_LOGV(mbEventChkTAG, "Register belongs to analog outputs table");
                    }

                    else if((reg_info.mb_offset >= 16) && (reg_info.mb_offset < 66)){
                        ESP_LOGV(mbEventChkTAG, "Register belongs to config table");
                        uint8_t index = reg_info.mb_offset - 16;
                        char key[5] = {'\0'};
                        sprintf(key, "C%u", index);

                        uint8_t writeFlag = 1;      // Write data to nvs by default

                        if (CFG_RUN_PGM && ((index > 0 ) && (index < 41)))  // Prohibited zone in Run mode
                            writeFlag = 0;

                        switch (index)
                        {
                        case 0:     // Run - Program mode register
                            if (CFG_RUN_PGM > 1)
                                read_nvs(key, &CFG_RUN_PGM);  // Don't accept invalid values
                            
                            break;

                        case 1:     // OP mode register
                            if (CFG_OP_MODE > 5)
                                writeFlag = 0;  // Don't accept invalid values
                            else 
                                if (CFG_RUN_PGM)
                                    writeFlag = 0;  // Don't change on run mode
                                else
                                    resetRequired = 1;  // Reset will be performed after saving
                            break;

                        // Add cases as they're needed...

                        case 12:        // CFG_DHCP (DHCP mode)
                            if (CFG_DHCP > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            else 
                                if (CFG_RUN_PGM)
                                    writeFlag = 0;  // Don't change on run mode
                                else
                                    set_DHCP();  // Configure DHCP (see function for details...)
                            break;

                        case 13:        //CFG_MB_MASTER_INTERFACE
                            if (CFG_MB_MASTER_INTERFACE > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            else
                                resetRequired = 1;
                            break;

                        case 14:        //CFG_MB_MASTER_BAUDRATE_H
                        case 15:        //CFG_MB_MASTER_BAUDRATE_L
                            resetRequired = 1;
                            break;

                        case 16:        //CFG_MB_SLAVE_INTERFACE
                            if (CFG_MB_SLAVE_INTERFACE > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            else
                                resetRequired = 1;
                            break;

                        case 17:        //CFG_MB_SLAVE_BAUDRATE_H
                        case 18:        //CFG_MB_SLAVE_BAUDRATE_L
                            resetRequired = 1;
                            break;

                        case 19:        //CFG_SLAVE_IP
                        case 20:
                            resetRequired = 1;
                            break;

                        case 21:        //CFG_GL_TMR_INTERVAL
                            if (CFG_GL_TMR_INTERVAL < 10)
                                writeFlag = 0;  // Don't accept invalid values
                            else
                                resetRequired = 1;
                            break;

                        case 22:        //CFG_GL_FILTER_ALPHA
                            if (CFG_GL_FILTER_ALPHA > 1000)
                                writeFlag = 0;  // Don't accept invalid values
                            break;

                        case 23:        //CFG_GL_PID_TMR_INTERVAL
                            if (CFG_GL_PID_TMR_INTERVAL < 10)
                                writeFlag = 0;  // Don't accept invalid values
                            else
                                resetRequired = 1;
                            break;

                        case 24:        // CFG_WIFI_MODE
                            if (CFG_WIFI_MODE > 3)
                                writeFlag = 0;
                            else
                                resetRequired = 1;
                            break; 

                        case 46:        //CFG_GL_PID_CP
                            if (CFG_GL_PID_CP > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            break;

                        case 47:        //CFG_GL_PID_CI
                            if (CFG_GL_PID_CI > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            break;

                        case 48:        //CFG_GL_PID_CD
                            if (CFG_GL_PID_CD > 1)
                                writeFlag = 0;  // Don't accept invalid values
                            break;

                        // Add cases as they're needed...

                        case 49:
                            if (CFG_REMOTA_LOG_LEVEL > 5)
                                writeFlag = 0;
                            else{
                                esp_log_level_set(TAG, CFG_REMOTA_LOG_LEVEL);
                                esp_log_level_set(mbSlaveTAG, CFG_REMOTA_LOG_LEVEL);
                                esp_log_level_set(mbEventChkTAG, CFG_REMOTA_LOG_LEVEL);
                            }
                            break;
                        
                        
                        default:
                            break;
                        }

                        if (writeFlag){
                             write_nvs(key, s3Tables.configTbl[0][index]); // Write new value to flash
                             if (resetRequired && (index == 0) && (CFG_RUN_PGM))    // Reset if OP mode was changed and run mode was selected
                                esp_restart();
                        }
                        else {
                             read_nvs(key, &s3Tables.configTbl[0][index]); // Restore previous value
                             writeFlag = 1;     // Restore flag
                        }
                        
                        
                    }

                    else if((reg_info.mb_offset >= 66) && (reg_info.mb_offset < 116)){
                        ESP_LOGV(mbEventChkTAG, "Register belongs to aux table");
                        uint8_t index = reg_info.mb_offset - 66;
                        char key[5] = {'\0'};
                        sprintf(key, "A%u", index);
                        switch (index)
                        {
                        case 43:  // AUX_RTC_YEAR
                            if (AUX_RTC_YEAR >= 1900) {
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        case 44:  // AUX_RTC_MONTH
                            if ((AUX_RTC_MONTH >= 1) && (AUX_RTC_MONTH <= 12)){
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        case 45:  // AUX_RTC_DAY
                            if ((AUX_RTC_DAY >= 1) && (AUX_RTC_DAY <= 31)){
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        case 46:  // AUX_RTC_HOUR
                            if (AUX_RTC_HOUR <= 23) {
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        case 47:  // AUX_RTC_MINUTE
                            if (AUX_RTC_MINUTE <= 59) {
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        case 48:  // AUX_RTC_SECOND
                            if (AUX_RTC_SECOND <= 59) {
                                if (setTime_ds1307() != ESP_OK)
                                    ESP_LOGE(TAG, "Failed to set RTC time");
                            }
                            break;
                        
                        default:
                            break;
                        }
                        

                    }

                    else if((reg_info.mb_offset >= 116) && (reg_info.mb_offset < 148)){
                        ESP_LOGV(mbEventChkTAG, "Register belongs to scaling factors table");
                        uint8_t index = (reg_info.mb_offset - 116) >> 1;
                        char key[6] = {'\0'};
                        sprintf(key, "SF%u", index);
                        if (CFG_RUN_PGM){ //Run mode
                            nvs_get_blob(app_nvs_handle, key, &s3Tables.scalingFactor[index], &size); //Changing not allowed in run mode
                        }
                        else{             //Program mode
                            nvs_set_blob(app_nvs_handle, key, &s3Tables.scalingFactor[index], size);
                        }
                        
                    }

                    else if((reg_info.mb_offset >= 148) && (reg_info.mb_offset < 180)){
                        ESP_LOGV(mbEventChkTAG, "Register belongs to scaling offsets table");
                        uint8_t index = (reg_info.mb_offset - 148) >> 1;
                        char key[6] = {'\0'};
                        sprintf(key, "SO%u", index);
                        if (CFG_RUN_PGM){ //Run mode
                            nvs_get_blob(app_nvs_handle, key, &s3Tables.scalingOffset[index], &size); //Changing not allowed in run mode
                        }
                        else{             //Program mode
                            nvs_set_blob(app_nvs_handle, key, &s3Tables.scalingOffset[index], size);
                        }
                    }
                }
            }         
        //}
    }
    
    taskYIELD();
}

void mb_master_poll_task(void *pvParameters){
    uint8_t type = 0;               //Type of parameter
    int cid;
    esp_err_t err;
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    uint8_t temp;
    
    while(!modbus_master_initialized){
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
    }
    
    while (1)
    {
        if (modbus_master_connected){
            switch (CFG_OP_MODE)
            {
            case 2:         //Mechanical Pump Wells
                // Get the information for characteristic cid from data dictionary
                cid = MP_ETM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&MP_IR_ETM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = MP_ITM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&MP_IR_ITM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ITM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = MP_JTM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&MP_IR_JTM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_JTM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                err = mbc_master_set_parameter(MP_SCM, "MP_SCM", (uint8_t*)&MP_HR_SCM, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }
                
                err = mbc_master_set_parameter(MP_HS1, "MP_HS1", (uint8_t*)&s3Tables.mbTbl8bit[0][0], &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }

                break;

            case 3:         //Electro Submersible Pump Wells
                // Get the information for characteristic cid from data dictionary
                cid = ES_ETM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_ETM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_ITM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_ITM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_ST1;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_ST1, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_TTM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_TTM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_WTM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_WTM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_WTB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_WTB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_ST2;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_ST2, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_YI1;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_YI1, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_YI2;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_YI2, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_SFM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_SFM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_PTSB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_PTSB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_PTDB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_PTDB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_TTSB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_TTSB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_TTDB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_TTDB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_VT;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&ES_IR_VT, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = ES_YI;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&s3Tables.mbTbl8bit[0][0], &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                err = mbc_master_set_parameter(ES_SCM, "ES_SCM", (uint8_t*)&ES_HR_SCM, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }
                
                temp = ES_COIL_HS1;
                err = mbc_master_set_parameter(ES_HS1, "ES_HS1", (uint8_t*)&temp, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }

                temp = ES_COIL_HS2;
                err = mbc_master_set_parameter(ES_HS2, "ES_HS2", (uint8_t*)&temp, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }
                break;

            case 4:         //Progressive Cavity Pump Wells
                cid = PC_WTM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_WTM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_WTB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_WTB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_ST2;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_ST2, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_YI1;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_YI1, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_YI2;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_YI2, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_SFM;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_SFM, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_PTSB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_PTSB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_PTDB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_PTDB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_TTSB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_TTSB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_TTDB;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_TTDB, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_VT;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&PC_IR_VT, &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                cid = PC_YI;
                err = mbc_master_get_cid_info(cid, &param_descriptor);
                if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key, (uint8_t*)&s3Tables.mbTbl8bit[0][0], &type);
                    if (err == ESP_OK) {
                        ESP_LOGV(TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        MP_IR_ETM);
                    } else {
                        modbus_master_connected = 0;
                        ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "Could not get information for characteristic %d.", cid);
                }

                err = mbc_master_set_parameter(PC_SCM, "PC_SCM", (uint8_t*)&PC_HR_SCM, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }
                
                temp = PC_COIL_HS1;
                err = mbc_master_set_parameter(PC_HS1, "PC_HS1", (uint8_t*)&temp, &type);
                if (err == ESP_OK) {
                    ESP_LOGV(TAG, "Set parameter data successfully.");
                } else {
                    modbus_master_connected = 0;
                    ESP_LOGE(TAG, "Set data fail, err = 0x%x (%s).", (int)err, (char*)esp_err_to_name(err));
                }
                break;
            
            }
        }
        else{
            mb_param_request_t req;
            req.slave_addr = 1;
            req.command = 1;
            req.reg_size = 1;
            req.reg_start = 0;
            uint8_t response;
            esp_err_t err = mbc_master_send_request(&req, &response);
            //ESP_LOGW(TAG, "Modbus master send request result: %i", err);
            if (err != ESP_ERR_INVALID_STATE)
                modbus_master_connected = 1;
            else
                modbus_master_connected = 0;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

void GLTimerCallBack(TimerHandle_t pxTimer){
    
    msCounter24 += CFG_GL_TMR_INTERVAL;

    // Filtering noise taking some measures and calculate an average value
    /* for (int i = 0; i < 40; i++){
        ftgl += GL_SV_FTGL;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ftgl = ftgl / 40; */

    // First order filter
    ftglFiltered = ftglFiltered + ((float)CFG_GL_FILTER_ALPHA/1000) * (GL_SV_FTGL - ftglFiltered);

    // Considering the last flow calculated and taking the average
    ftglFiltered = (last_ftgl + ftglFiltered)/2;
    last_ftgl = ftglFiltered;

    // Calculate estimated volume for 24H
    GL_FQ24_PROY = ftglFiltered;

    // Accumulate gas volume in the timer interval
    float accum = (ftglFiltered / 86400) *  (float)CFG_GL_TMR_INTERVAL / 1000;
    GL_FQ24 += accum;  

    if (msCounter24 >= MS24H){
        GL_FQ72 = GL_FQ48;
        GL_FQ48 = GL_FQ24;
        GL_FQ24 = 0;
        msCounter24 = 0;
    }
}

void GLTimerPIDCallBack(TimerHandle_t pxTimer){
    time2 = esp_timer_get_time();
    s3Tables.auxTbl[0][49] = (time2 - time1)/1000;
    
    // Error calculation
    PID_e = CFG_GL_PID_SP - GL_AI_FTGL;

    // Proportional Gain (Gp)
    PID_up = CFG_GL_PID_KP * PID_e;
    
    // Integral Gain (Gi)
    PID_ui = lastPID_ui + (float)CFG_GL_PID_KI * PID_e * ((float)CFG_GL_PID_TMR_INTERVAL / 1000);
    
    lastPID_ui = PID_ui; // Keep track of last ui

    // Derivative Control (with N Filter) (Gd)
    PID_ud = ( PID_ud + CFG_GL_PID_KD * CFG_GL_PID_N * (PID_e - lastPID_e) ) / ( 1 + CFG_GL_PID_N * ((float)CFG_GL_PID_TMR_INTERVAL / 1000));
    
    lastPID_e = PID_e; // Keep track of last error vaue

    // PID output:
    PID_u.floatValue = CFG_GL_PID_CP * PID_up + CFG_GL_PID_CI * PID_ui + CFG_GL_PID_CD * PID_ud;
    GL_AO_FCV_L = PID_u.uint16Values.low;
    GL_AO_FCV_H = PID_u.uint16Values.high;    

    time1 = esp_timer_get_time();
}

esp_err_t init_nvs(void){
    esp_err_t r;
    nvs_flash_init();
    r = nvs_open("Main_Namespace", NVS_READWRITE, &app_nvs_handle);
    return r;
}

esp_err_t read_nvs(char *key, uint16_t *value){
    esp_err_t r;
    r = nvs_get_u16(app_nvs_handle, key, value);
    return r;
}

esp_err_t write_nvs(char *key, uint16_t value){
    esp_err_t r;
    r = nvs_set_u16(app_nvs_handle, key, value);
    return r;
}

esp_err_t create_table_nvs(char *c, uint8_t tableSize){
    esp_err_t r = ESP_OK;
    uint16_t value;
    char key[5] = {'\0'};
    sprintf(key, "%s0", c);
    //printf("%s", key);
    r = nvs_get_u16(app_nvs_handle, key, &value);
    if (r == ESP_ERR_NVS_NOT_FOUND)
    {
        for (int i = 0; i < tableSize; i++)
        {
            sprintf(key, "%s%i", c, i);
            printf("%s ", key);
            r = nvs_set_u16(app_nvs_handle, key, 0);
            if (r != ESP_OK)
                return r;
        }
        printf("\n");
    }
    else
        ESP_LOGI(TAG, "The table %s already exist in nvs namespace", c);
    return r;
}

esp_err_t create_float_table_nvs(char *c, uint8_t tableSize){
    esp_err_t r = ESP_OK;
    float value = 0;
    size_t size = sizeof(float);
    char key[5] = {'\0'};
    sprintf(key, "%s0", c);
    //printf("%s", key);
    r = nvs_get_blob(app_nvs_handle, key, &value, &size);
    if (r == ESP_ERR_NVS_NOT_FOUND)
    {
        for (int i = 0; i < tableSize; i++)
        {
            sprintf(key, "%s%i", c, i);
            printf("%s ", key);
            r = nvs_set_blob(app_nvs_handle, key, &value, size);
            if (r != ESP_OK)
                return r;
        }
        printf("\n");
    }
    else
        ESP_LOGI(TAG, "The table %s already exist in nvs namespace", c);
    return r;
}

esp_err_t set_configDefaults_nvs(void){
    ESP_LOGW(TAG, "Setting nvs flash contents to default values...");

    //Create tables in the nvs namespace (if they don't exist):
    create_table_nvs("C", s3Tables.configSize);     //For config table
    create_table_nvs("A", s3Tables.auxSize);        //For aux table
    create_float_table_nvs("SF", s3Tables.anSize);        //For Scaling factors table
    create_float_table_nvs("SO", s3Tables.anSize);        //For Scaling offsets table

    write_nvs("C0",  0x0001);    //40017
    write_nvs("C1",  0x0000);    //40018
    write_nvs("C2",  0xAC10);    //40019
    write_nvs("C3",  0x0064);    //40020
    write_nvs("C4",  0x0000);    //40021
    write_nvs("C5",  0x0000);    //40022
    write_nvs("C6",  0x0000);    //40023
    write_nvs("C7",  0x0000);    //40024
    write_nvs("C8",  0x0000);    //40025
    write_nvs("C9",  0x0000);    //40026
    write_nvs("C10", 0xAC10);    //40027
    write_nvs("C11", 0x0001);    //40028
    write_nvs("C12", 0x0000);    //40029
    write_nvs("C13", 0x0000);    //40030
    write_nvs("C14", 0x0001);    //40031
    write_nvs("C15", 0xC200);    //40032
    write_nvs("C16", 0x0001);    //40033
    write_nvs("C17", 0x0001);    //40034
    write_nvs("C18", 0xF400);    //40035
    write_nvs("C19", 0xAC10);    //40036
    write_nvs("C20", 0x0004);    //40037
    write_nvs("C21", 0x000A);    //40038
    write_nvs("C22", 0x000A);    //40039

    write_nvs("C23", 0x0000);    //40040
    write_nvs("C24", 0x0000);    //40041
    write_nvs("C25", 0x0000);    //40042
    write_nvs("C26", 0x0000);    //40043
    write_nvs("C27", 0x0000);    //40044
    write_nvs("C28", 0x0000);    //40045
    write_nvs("C29", 0x0000);    //40046
    write_nvs("C30", 0x0000);    //40047
    write_nvs("C31", 0x0000);    //40048
    write_nvs("C32", 0x0000);    //40049
    write_nvs("C33", 0x0000);    //40050
    write_nvs("C34", 0x0000);    //40051
    write_nvs("C35", 0x0000);    //40052
    write_nvs("C36", 0x0000);    //40053
    write_nvs("C37", 0x0000);    //40054
    write_nvs("C38", 0x0000);    //40055
    write_nvs("C39", 0x0000);    //40056
    write_nvs("C40", 0x0000);    //40057
    write_nvs("C41", 0x0000);    //40058
    write_nvs("C42", 0x0000);    //40059
    write_nvs("C43", 0x0000);    //40060
    write_nvs("C44", 0x0000);    //40061
    write_nvs("C45", 0x0000);    //40062
    write_nvs("C46", 0x0000);    //40063
    write_nvs("C47", 0x0000);    //40064
    write_nvs("C48", 0x0000);    //40065

    write_nvs("C49", 0x0004);    //40066

    ESP_LOGW(TAG, "Default values has been written to flash");
    return ESP_OK;
}

esp_err_t init_FAT_fileSystem(void){
    // Mount path for the partition
    const char *base_path = "/spiflash";

    // Handle of the wear levelling library instance
    static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

    // Register and mount FAT partition:
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = false,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err;
    err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return err;
    }

    //Get info about FAT partition:
    uint64_t total = 0, free = 0;
    err = esp_vfs_fat_info(base_path, &total, &free);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get partition info (%s)", esp_err_to_name(err));
        return err;
    }
    else {
        ESP_LOGI(TAG, "Partition size: Total %llu, free %llu", total, free);
    }
    return ESP_OK;
}

esp_err_t ds1307_init(void){

    ESP_ERROR_CHECK(i2cdev_init());
    
    memset(&dev, 0, sizeof(i2c_dev_t));
    esp_err_t res;
    
    dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    
    res = ds1307_init_desc(&dev, 0, 39, 40);
    
    // Read info from RTC:
    struct tm actualTime;
    ds1307_get_time(&dev, &actualTime);
    //Write info back to RTC: (This is for ensure that RTC is running after init...)
    ds1307_set_time(&dev, &actualTime);
    return res;
}

esp_err_t setTime_ds1307(void){
    // setup datetime: 2023-10-20 20:30:10
    uint16_t d, m, y;
    d = AUX_RTC_DAY;
    m = AUX_RTC_MONTH;
    y = AUX_RTC_YEAR;

    struct tm actualTime = {
        .tm_year = AUX_RTC_YEAR - 1900, //since 1900 (2023 - 1900)
        .tm_mon  = AUX_RTC_MONTH - 1,  // 0-based
        .tm_mday = AUX_RTC_DAY,
        .tm_hour = AUX_RTC_HOUR,
        .tm_min  = AUX_RTC_MINUTE,
        .tm_sec  = AUX_RTC_SECOND,
        .tm_wday =  (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7
    };
    esp_err_t res = ds1307_set_time(&dev, &actualTime);
    return res;
}

esp_err_t system_logInput(const char* message){
    // Get the actual timestamp:
    struct tm actualTime;
    ds1307_get_time(&dev, &actualTime);
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%d-%m-%Y %H:%M:%S", &actualTime);
    
    ESP_LOGV(TAG, "Creating a backup file from sys_log.log to sys_log.bak");
    FILE *file, *f_backup;
    char character;

    // Open sys_log.log for reading, if doesn't exists, create it:
    file = fopen("/spiflash/sys_log.log", "rb");
    if (file == NULL) {
        ESP_LOGV(TAG, "File does not exist, creating...");
        file = fopen("/spiflash/sys_log.log", "wb");
        if (file == NULL) {
            ESP_LOGV(TAG, "Failed to create file");
            return ESP_FAIL;
        }
        fclose(file);
        file = fopen("/spiflash/sys_log.log", "rb");
    }

    // Open sys_log.bak for writing:
    f_backup = fopen("/spiflash/sys_log.bak", "wb");
    if (f_backup == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        fclose(file);
        return ESP_FAIL;
    }

    // Read the content of sys_log.log and copy to sys_log.bak:
    while (1) {
        character = fgetc(file);
        if (feof(file)) {
            break; // End of original file
        }
        fputc(character, f_backup);
    }
    fclose(file);
    fclose(f_backup);

    //Append timestamp and message to sys_log.bak:
    ESP_LOGV(TAG, "Append info to the file sys_log.bak");
    file = fopen("/spiflash/sys_log.bak", "a");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for append");
        return ESP_FAIL;
    }
    
    fprintf(file, "(%s) - %s\n", time_str, message);
    fclose(file);

    //Remove original file:
    ESP_LOGV(TAG, "Removing original file sys_log.log");
    if (remove("/spiflash/sys_log.log") == 0)
        ESP_LOGV(TAG, "Original file has been removed");
    else
        ESP_LOGE(TAG, "Failed to remove original file");

    //Rename sys_log.bak file:
    ESP_LOGV(TAG, "Rename the file sys_log.bak to sys_log.log");
    if (rename("/spiflash/sys_log.bak", "/spiflash/sys_log.log") == 0)
        ESP_LOGV(TAG, "File renamed succesfully");
    else {
        ESP_LOGE(TAG, "Failed to remove original file");
        return ESP_FAIL;
    }
        

    return ESP_OK;
}

esp_err_t print_systemLog(void){
    printf("\nSystem log (sys_log.log file):\n\n");
    //Open a text file for reading
    FILE *f;
    f = fopen("/spiflash/sys_log.log", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[100];
    while (fgets(line, sizeof(line), f)){
        printf("%s", line);
    }
    printf("\n");
    fclose(f);

    return ESP_OK;
}
