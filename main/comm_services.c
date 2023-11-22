/*
______________________________________________________________________________________________________
                                 Communication Services source file
                                        by: Javier Ruzzante C
                                            (Aug 2023)
    ______________________________________________________________________________________________________
*/

//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "remota_globals.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________
//Comment out this line to use dynamic IP, via DHCP Server:

//#define ETHERNET_USE_STATIC_IP

//#ifdef ETHERNET_USE_STATIC_IP

/* #define ETHERNET_IP_ADDR        "192.168.1.20"
#define ETHERNET_GATEWAY        "192.168.1.10"
#define ETHERNET_SUBNET_MASK    "255.255.255.0" */

/* #define ETHERNET_IP_ADDR        "172.16.0.100"
#define ETHERNET_GATEWAY        "172.16.0.1"
#define ETHERNET_SUBNET_MASK    "255.255.255.0" */

//#endif //ETHERNET_USE_STATIC_IP

//Comment out this line to use ENC28J60 Ethernet Module

#define ETHERNET_USE_W5500  

//  Important Note:
// Enable support for W5500 via menu-config:
// Activate "Use W5500 (MAC RAW)" option along with 
// "Support SPI to Ethernet Module" in the Ethernet section!

#ifndef ETHERNET_USE_W5500
#include "esp_eth_enc28j60.h"
#endif

#define ENC28J60_DUPLEX_FULL 0

#define ETHERNET_MISO_GPIO 13
#define ETHERNET_MOSI_GPIO 11
#define ETHERNET_SCLK_GPIO 12
#define ETHERNET_CS_GPIO   10
#define ETHERNET_INT_GPIO  14

#define ETHERNET_SPI_CLOCK_MHZ 16
#define ETHERNET_SPI_HOST SPI3_HOST

//____________________________________________________________________________________________________
// Function prototypes:
//____________________________________________________________________________________________________
esp_err_t ethernetInit(void);
void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);
void wifi_event_handler(void *arg, esp_event_base_t event_base, 
                              int32_t event_id, void *event_data);
void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data);
void set_ip_eth0(void);
void set_wifi_STA_ip(void);
void set_wifi_AP_ip(void);

esp_err_t WiFi_Begin_STA(void);
esp_err_t WiFi_Begin_STA_AP(void);
esp_err_t WiFi_Begin_AP(void);
esp_err_t WiFi_init(void);

//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________
static const char *ethTAG = "Ethernet";
esp_netif_t *eth_netif = NULL;
esp_netif_t *wifi_STA_netif = NULL;
esp_netif_t *wifi_AP_netif = NULL;

//____________________________________________________________________________________________________
// Function implementations:
//____________________________________________________________________________________________________

/** Event handler for Ethernet events */
void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(ethTAG, "Ethernet Link Up");
        ESP_LOGI(ethTAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        system_logInput("Ethernet module link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(ethTAG, "Ethernet Link Down");
        system_logInput("Ethernet module link down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(ethTAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(ethTAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(ethTAG, "Ethernet Got IP Address");
    ESP_LOGI(ethTAG, "~~~~~~~~~~~");
    ESP_LOGI(ethTAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(ethTAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(ethTAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(ethTAG, "~~~~~~~~~~~");
    ethernet_got_ip = 1;
    char line[50];
    sprintf(line, "Ethernet got IP: " IPSTR, IP2STR(&ip_info->ip));
    system_logInput(line);
}

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        WiFi_Status = WIFI_STARTED;
        ESP_LOGI(wifiTAG, "Connecting to AP...");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        WiFi_Status = WIFI_CONNECTED;
        ESP_LOGI(wifiTAG, "Connected to AP");
        system_logInput("WiFi station: Connected to AP");
        break;
    case IP_EVENT_STA_GOT_IP:
        WiFi_Status = WIFI_GOT_IP;
        ESP_LOGI(wifiTAG, "Got IP address");
        wifi_got_ip = 1;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        char line[50];
        sprintf(line, "WiFi station got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        system_logInput(line);
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        WiFi_Status = WIFI_DISCONNECTED;
        ESP_LOGI(wifiTAG, "Disconnected from AP");
        system_logInput("WiFi station: Disconnected from AP");
        esp_wifi_connect();
        break;

    case WIFI_EVENT_AP_START:
        WiFi_Status = WIFI_AP_STARTED;
        ESP_LOGI(wifiTAG, "WIFI_EVENT_AP_START");
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        ESP_LOGI(wifiTAG, "WIFI_EVENT_AP_STADISCONNECTED");
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(wifiTAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        system_logInput("WiFi AP: Station disconnected");
    }
    break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
        ESP_LOGI(wifiTAG, "WIFI_EVENT_AP_STACONNECTED");
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(wifiTAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        system_logInput("WiFi AP: Station connected");
    }
    break;

    default:
        break;
    }
}

esp_err_t ethernetInit(void) {

#ifndef ETHERNET_USE_W5500

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));     //This is already done in the SPI handshaking config

    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    if (!CFG_DHCP){
        esp_netif_dhcpc_stop(eth_netif);

        set_ip_eth0();
        //esp_netif_ip_info_t ip_info;

        /* char IP0[16] = {'\0'};  
        sprintf(IP0, "%hhu.%hhu.%hhu.%hhu", *CFG_IP0, *(CFG_IP0+1), *(CFG_IP0+2), *(CFG_IP0+3)); */
        /* char GW[16] = {'\0'};  
        sprintf(GW, "%hhu.%hhu.%hhu.%hhu", *CFG_GW, *(CFG_GW+1), *(CFG_GW+2), *(CFG_GW+3)); */

        //char IP0_str[16] = {'\0'};
        //uint8_t IP0[4] = {0};
        //IP0[0] = *CFG_IP0 >> 8;
        //IP0[1] = *CFG_IP0 & 0x00FF;
        //IP0[2] = *(CFG_IP0+1) >> 8;
        //IP0[3] = *(CFG_IP0+1) & 0x00FF;
        //sprintf(IP0_str, "%hhu.%hhu.%hhu.%hhu", IP0[0], IP0[1], IP0[2], IP0[3]);

        //char GW_str[16] = {'\0'};
        //uint8_t GW[4] = {0};
        //GW[0] = *CFG_GW >> 8;
        //GW[1] = *CFG_GW & 0x00FF;
        //GW[2] = *(CFG_GW+1) >> 8;
        //GW[3] = *(CFG_GW+1) & 0x00FF;
        //sprintf(GW_str, "%hhu.%hhu.%hhu.%hhu", GW[0], GW[1], GW[2], GW[3]);

        //esp_netif_str_to_ip4(IP0_str, &ip_info.ip);          //Set IP address
        //esp_netif_str_to_ip4(GW_str, &ip_info.gw);          //Set Gateway
        //esp_netif_str_to_ip4(ETHERNET_SUBNET_MASK, &ip_info.netmask);    //Set Subnet Mask

        //esp_netif_set_ip_info(eth_netif, &ip_info);
    }
    

    spi_bus_config_t buscfg = {
        .miso_io_num = ETHERNET_MISO_GPIO,
        .mosi_io_num = ETHERNET_MOSI_GPIO,
        .sclk_io_num = ETHERNET_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETHERNET_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    /* ENC28J60 ethernet driver is based on spi driver */
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = ETHERNET_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ETHERNET_CS_GPIO,
        .queue_size = 20,
        .cs_ena_posttrans = enc28j60_cal_spi_cs_hold_time(ETHERNET_SPI_CLOCK_MHZ),
    };

    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(ETHERNET_SPI_HOST, &spi_devcfg);
    enc28j60_config.int_gpio_num = ETHERNET_INT_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
    phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    /* ENC28J60 doesn't burn any factory MAC address, we need to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    mac->set_addr(mac, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    });

    // ENC28J60 Errata #1 check
    if (emac_enc28j60_get_chip_info(mac) < ENC28J60_REV_B5 && ETHERNET_SPI_CLOCK_MHZ < 8) {
        ESP_LOGE(ethTAG, "SPI frequency must be at least 8 MHz for chip revision less than 5");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* It is recommended to use ENC28J60 in Full Duplex mode since multiple errata exist to the Half Duplex mode */
#if ENC28J60_DUPLEX_FULL
    eth_duplex_t duplex = ETH_DUPLEX_FULL;
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_DUPLEX_MODE, &duplex));
#endif

    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

#endif //ETHERNET_USE_W5500

#ifdef ETHERNET_USE_W5500
    // Initialize TCP/IP network interface (should be called only once in application)
    if (esp_netif_init() != ESP_OK)
        return ESP_FAIL;
    // Create default event loop that running in background

    esp_event_loop_create_default();
    
    // Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    //esp_netif_t *eth_netif_spi = NULL;
    
    esp_netif_config.if_key = "ETH_SPI";
    esp_netif_config.if_desc = "eth";
    esp_netif_config.route_prio = 30;
    eth_netif = esp_netif_new(&cfg_spi);

    if (!CFG_DHCP){     //Static IP Configuration:
        esp_netif_dhcpc_stop(eth_netif);
        set_ip_eth0();
    }


    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();

    // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
    //gpio_install_isr_service(0);        //This is already done in the SPI handshaking config

    // Init SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = ETHERNET_MISO_GPIO,
        .mosi_io_num = ETHERNET_MOSI_GPIO,
        .sclk_io_num = ETHERNET_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    if (spi_bus_initialize(ETHERNET_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK)
        return ESP_FAIL;

    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    
    esp_eth_mac_t *mac_spi;
    esp_eth_phy_t *phy_spi;
    esp_eth_handle_t eth_handle_spi = NULL;

    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = ETHERNET_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20
    };
    // Set SPI module Chip Select GPIO
    spi_devcfg.spics_io_num = ETHERNET_CS_GPIO;
    // Set remaining GPIO numbers and configuration used by the SPI module
    phy_config_spi.phy_addr = 1;
    phy_config_spi.reset_gpio_num = -1;

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETHERNET_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = ETHERNET_INT_GPIO;
    mac_spi = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
    phy_spi = esp_eth_phy_new_w5500(&phy_config_spi);

    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi, phy_spi);
    if (esp_eth_driver_install(&eth_config_spi, &eth_handle_spi) != ESP_OK)
        return ESP_FAIL;

    /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
    02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    if (esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    }))
        return ESP_FAIL;

    // attach Ethernet driver to TCP/IP stack
    if (esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle_spi)))
        return ESP_FAIL;

    // Register user defined event handers
    if (esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL))
        return ESP_FAIL;
    if (esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL))
        return ESP_FAIL;

    /* start Ethernet driver state machine */
    if (esp_eth_start(eth_handle_spi))
        return ESP_FAIL;
#endif //ETHERNET_USE_W5500

    return ESP_OK;
}

void set_ip_eth0(void){
    esp_netif_ip_info_t ip_info;

    /* char IP0[16] = {'\0'};  
    sprintf(IP0, "%hhu.%hhu.%hhu.%hhu", *CFG_IP0, *(CFG_IP0+1), *(CFG_IP0+2), *(CFG_IP0+3)); */
    /* char GW[16] = {'\0'};  
    sprintf(GW, "%hhu.%hhu.%hhu.%hhu", *CFG_GW, *(CFG_GW+1), *(CFG_GW+2), *(CFG_GW+3)); */

    char IP0_str[16] = {'\0'};
    uint8_t IP0[4] = {0};
    IP0[0] = *CFG_IP0 >> 8;
    IP0[1] = *CFG_IP0 & 0x00FF;
    IP0[2] = *(CFG_IP0+1) >> 8;
    IP0[3] = *(CFG_IP0+1) & 0x00FF;
    sprintf(IP0_str, "%hhu.%hhu.%hhu.%hhu", IP0[0], IP0[1], IP0[2], IP0[3]);

    char GW_str[16] = {'\0'};
    uint8_t GW[4] = {0};
    GW[0] = *CFG_GW >> 8;
    GW[1] = *CFG_GW & 0x00FF;
    GW[2] = *(CFG_GW+1) >> 8;
    GW[3] = *(CFG_GW+1) & 0x00FF;
    sprintf(GW_str, "%hhu.%hhu.%hhu.%hhu", GW[0], GW[1], GW[2], GW[3]);

    char subMask_str[16] = {'\0'};
    uint8_t subMask[4] = {0};
    subMask[0] = *CFG_SUBNET_MASK >> 8;
    subMask[1] = *CFG_SUBNET_MASK & 0x00FF;
    subMask[2] = *(CFG_SUBNET_MASK+1) >> 8;
    subMask[3] = *(CFG_SUBNET_MASK+1) & 0x00FF;
    sprintf(subMask_str, "%hhu.%hhu.%hhu.%hhu", subMask[0], subMask[1], subMask[2], subMask[3]);

    esp_netif_str_to_ip4(IP0_str, &ip_info.ip);          //Set IP address
    esp_netif_str_to_ip4(GW_str, &ip_info.gw);          //Set Gateway
    esp_netif_str_to_ip4(subMask_str, &ip_info.netmask);    //Set Subnet Mask

    esp_netif_set_ip_info(eth_netif, &ip_info);
}

void set_wifi_STA_ip(void){
    esp_netif_ip_info_t ip_info;

    char IP2_str[16] = {'\0'};
    uint8_t IP2[4] = {0};
    IP2[0] = *CFG_IP2 >> 8;
    IP2[1] = *CFG_IP2 & 0x00FF;
    IP2[2] = *(CFG_IP2+1) >> 8;
    IP2[3] = *(CFG_IP2+1) & 0x00FF;
    sprintf(IP2_str, "%hhu.%hhu.%hhu.%hhu", IP2[0], IP2[1], IP2[2], IP2[3]);

    char GW_str[16] = {'\0'};
    uint8_t GW[4] = {0};
    GW[0] = *CFG_GW >> 8;
    GW[1] = *CFG_GW & 0x00FF;
    GW[2] = *(CFG_GW+1) >> 8;
    GW[3] = *(CFG_GW+1) & 0x00FF;
    sprintf(GW_str, "%hhu.%hhu.%hhu.%hhu", GW[0], GW[1], GW[2], GW[3]);

    char subMask_str[16] = {'\0'};
    uint8_t subMask[4] = {0};
    subMask[0] = *CFG_SUBNET_MASK >> 8;
    subMask[1] = *CFG_SUBNET_MASK & 0x00FF;
    subMask[2] = *(CFG_SUBNET_MASK+1) >> 8;
    subMask[3] = *(CFG_SUBNET_MASK+1) & 0x00FF;
    sprintf(subMask_str, "%hhu.%hhu.%hhu.%hhu", subMask[0], subMask[1], subMask[2], subMask[3]);

    esp_netif_str_to_ip4(IP2_str, &ip_info.ip);          //Set IP address
    esp_netif_str_to_ip4(GW_str, &ip_info.gw);          //Set Gateway
    esp_netif_str_to_ip4(subMask_str, &ip_info.netmask);    //Set Subnet Mask

    //esp_netif_dhcpc_stop(wifi_STA_netif);
    esp_netif_set_ip_info(wifi_STA_netif, &ip_info);
}

void set_wifi_AP_ip(void){
    esp_netif_ip_info_t ip_info;

    char IP3_str[16] = {'\0'};
    uint8_t IP3[4] = {0};
    IP3[0] = *CFG_IP3 >> 8;
    IP3[1] = *CFG_IP3 & 0x00FF;
    IP3[2] = *(CFG_IP3+1) >> 8;
    IP3[3] = *(CFG_IP3+1) & 0x00FF;
    sprintf(IP3_str, "%hhu.%hhu.%hhu.%hhu", IP3[0], IP3[1], IP3[2], IP3[3]);

    char subMask_str[16] = {'\0'};
    uint8_t subMask[4] = {0};
    subMask[0] = *CFG_SUBNET_MASK >> 8;
    subMask[1] = *CFG_SUBNET_MASK & 0x00FF;
    subMask[2] = *(CFG_SUBNET_MASK+1) >> 8;
    subMask[3] = *(CFG_SUBNET_MASK+1) & 0x00FF;
    sprintf(subMask_str, "%hhu.%hhu.%hhu.%hhu", subMask[0], subMask[1], subMask[2], subMask[3]);

    esp_netif_str_to_ip4(IP3_str, &ip_info.ip);          //Set IP address
    esp_netif_str_to_ip4(IP3_str, &ip_info.gw);          //Set Gateway
    esp_netif_str_to_ip4(subMask_str, &ip_info.netmask);    //Set Subnet Mask

    esp_netif_set_ip_info(wifi_AP_netif, &ip_info);
    char line[50];
    sprintf(line, "WiFi AP IP address: %s", IP3_str);
    system_logInput(line);
}

void set_DHCP(void){
    if (CFG_DHCP){
        esp_netif_dhcpc_start(eth_netif);
        if ((CFG_WIFI_MODE == 0) || (CFG_WIFI_MODE == 2))
            esp_netif_dhcpc_start(wifi_STA_netif);
        ESP_LOGI(TAG, "DHCP Started");
    }

    else{
        esp_netif_dhcpc_stop(eth_netif);
        if ((CFG_WIFI_MODE == 0) || (CFG_WIFI_MODE == 2))
            esp_netif_dhcpc_stop(wifi_STA_netif);
        ESP_LOGI(TAG, "DHCP Stopped");
        set_ip_eth0();
        if ((CFG_WIFI_MODE == 0) || (CFG_WIFI_MODE == 2))
            set_wifi_STA_ip();
        ESP_LOGI(TAG, "Static IP config applied");
    }
}

esp_err_t WiFi_Begin_STA(void)
{
    nvs_flash_init();

    // Stage 1. Wi-Fi/LwIP Init Phase:

     //esp_netif_init();
     //esp_event_loop_create_default();
     wifi_STA_netif= esp_netif_create_default_wifi_sta();

     if (!CFG_DHCP){     //Static IP Configuration:
        esp_netif_dhcpc_stop(wifi_STA_netif);
        set_wifi_STA_ip();
    }

     wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
     esp_wifi_init(&wifi_init_cfg);

    #ifdef WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_sta_config_t wifi_sta_cfg = {};

    strcpy((char *)wifi_sta_cfg.ssid, WIFI_SSID);
    strcpy((char *)wifi_sta_cfg.password, WIFI_PASSWORD);

    wifi_config_t wifi_cfg = {.sta = wifi_sta_cfg};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));

    #ifdef WIFI_STA_CUSTOM_MAC
    uint8_t customMac[] = JRC_WIFI_STA_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_STA, customMac);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

esp_err_t WiFi_Begin_AP(void)
{
    nvs_flash_init();

    // Stage 1. Wi-Fi/LwIP Init Phase:

    //esp_netif_init();
    //esp_event_loop_create_default();
    wifi_AP_netif = esp_netif_create_default_wifi_ap();

    esp_netif_dhcps_stop(wifi_AP_netif);
    set_wifi_AP_ip();
    esp_netif_dhcps_start(wifi_AP_netif);

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);

    #ifdef JRC_WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    strcpy((char *)wifi_ap_config.ap.ssid, WIFI_AP_SSID);
    strcpy((char *)wifi_ap_config.ap.password, WIFI_AP_PASSWORD);

    if (strlen(WIFI_AP_PASSWORD) == 0)
    {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    #ifdef JRC_WIFI_AP_CUSTOM_MAC
    uint8_t customMac[] = JRC_WIFI_AP_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_AP, customMac);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return ESP_OK;
}

esp_err_t WiFi_Begin_STA_AP(void)
{
    nvs_flash_init();

    // Stage 1. Wi-Fi/LwIP Init Phase:

    //esp_netif_init();
    //esp_event_loop_create_default();
    wifi_STA_netif = esp_netif_create_default_wifi_sta();
    wifi_AP_netif = esp_netif_create_default_wifi_ap();

    if (!CFG_DHCP){     //Static IP Configuration:
        esp_netif_dhcpc_stop(wifi_STA_netif);
        set_wifi_STA_ip();
    }

    esp_netif_dhcps_stop(wifi_AP_netif);
    set_wifi_AP_ip();
    esp_netif_dhcps_start(wifi_AP_netif);

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);

    #ifdef JRC_WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_sta_config_t wifi_sta_cfg = {};

    strcpy((char *)wifi_sta_cfg.ssid, WIFI_SSID);
    strcpy((char *)wifi_sta_cfg.password, WIFI_PASSWORD);

    //printf("\n%s\n%s\n", wifi_sta_cfg.ssid, wifi_sta_cfg.password);

    wifi_ap_config_t wifi_ap_cfg = {
        .ssid_len = strlen(WIFI_AP_SSID),
        .channel = 0,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {
            .required = false,
        },
    };

    strcpy((char *)wifi_ap_cfg.ssid, WIFI_AP_SSID);
    strcpy((char *)wifi_ap_cfg.password, WIFI_AP_PASSWORD);

    //printf("\n%s\n%s\n", wifi_ap_cfg.ssid, wifi_ap_cfg.password);

    if (strlen(WIFI_AP_PASSWORD) == 0)
    {
        wifi_ap_cfg.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config_t wifi_sta_config = {.sta = wifi_sta_cfg};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    
    #ifdef JRC_WIFI_STA_CUSTOM_MAC
    uint8_t customMacSta[] = JRC_WIFI_STA_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_STA, customMacSta);
    #endif

    wifi_config_t wifi_ap_config = {.ap = wifi_ap_cfg};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    #ifdef JRC_WIFI_AP_CUSTOM_MAC
    uint8_t customMacAP[] = JRC_WIFI_AP_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_AP, customMacAP);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

esp_err_t WiFi_init(void)
{
    // WiFi Initialization:
    if (CFG_WIFI_MODE < 3) {
        /* while (xSemaphoreTake(sysLogFileSem, portMAX_DELAY) != pdTRUE)
            continue; */

        FILE *file;
        file = fopen("/spiflash/wifi.cfg", "rb"); 
        if (file == NULL) {
            ESP_LOGE(TAG, "Failed to open wifi.cfg");
            return ESP_FAIL;
        }

        char line[25]={'\0'};
        uint8_t i = 0;

        fgets(line, sizeof(line), file);
        i = 0;
        while (1)
        {
            if ((line[i] == '\r') || (line[i] == '\n') || (line[i] == '\0'))
                break;
            WIFI_SSID[i] = line[i];
            i++;
        }
        
        fgets(line, sizeof(line), file);
        i = 0;
        while (1)
        {
            if ((line[i] == '\r') || (line[i] == '\n') || (line[i] == '\0'))
                break;
            WIFI_PASSWORD[i] = line[i];
            i++;
        }

        fgets(line, sizeof(line), file);
        i = 0;
        while (1)
        {
            if ((line[i] == '\r') || (line[i] == '\n') || (line[i] == '\0'))
                break;
            WIFI_AP_SSID[i] = line[i];
            i++;
        }

        fgets(line, sizeof(line), file);
        i = 0;
        while (1)
        {
            if ((line[i] == '\r') || (line[i] == '\n') || (line[i] == '\0'))
                break;
            WIFI_AP_PASSWORD[i] = line[i];
            i++;
        }

        fclose(file);

        //xSemaphoreGive(sysLogFileSem);

        //printf("\n%s\n%s\n%s\n%s\n", WIFI_SSID, WIFI_PASSWORD, WIFI_AP_SSID, WIFI_AP_PASSWORD);
    }
    
    esp_err_t res = ESP_OK;

    switch (CFG_WIFI_MODE)
    {
    case 0:             // Init WiFi as station mode
        ESP_LOGI(wifiTAG, "WiFi initialization in station mode");
        system_logInput("WiFi initialization in station mode");
        res = WiFi_Begin_STA();
        break;
    case 1:             // Init WiFi as AP mode
        ESP_LOGI(wifiTAG, "WiFi initialization in AP mode");
        res = WiFi_Begin_AP();
        system_logInput("WiFi initialization in AP mode");
        break;
    case 2:             // Init WiFi as station + AP mode
        ESP_LOGI(wifiTAG, "WiFi initialization in station + AP mode");
        system_logInput("WiFi initialization in station + AP mode");
        res = WiFi_Begin_STA_AP();
        break;
    case 3:
        ESP_LOGW(wifiTAG, "WiFi mode is set to 3 --> (No WiFi mode)");
        break;
    
    default:
        break;
    }

    return res;
}