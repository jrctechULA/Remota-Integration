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
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "esp_log.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________
//Comment out this line to use dynamic IP, via DHCP Server:

//#define ETHERNET_USE_STATIC_IP

#ifdef ETHERNET_USE_STATIC_IP

#define ETHERNET_IP_ADDR        "192.168.1.20"
#define ETHERNET_GATEWAY        "192.168.1.10"
#define ETHERNET_SUBNET_MASK    "255.255.255.0"

#endif //ETHERNET_USE_STATIC_IP

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
/* void ethernetInit(void);
void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);
void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data); */

//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________
static const char *ethTAG = "Ethernet";
esp_netif_t *eth_netif = NULL;

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
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(ethTAG, "Ethernet Link Down");
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
}

void ethernetInit() {

#ifndef ETHERNET_USE_W5500

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));     //This is already done in the SPI handshaking config

    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    #ifdef ETHERNET_USE_STATIC_IP

    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ip_info;

    esp_netif_str_to_ip4(ETHERNET_IP_ADDR, &ip_info.ip);          //Set IP address
    esp_netif_str_to_ip4(ETHERNET_GATEWAY, &ip_info.gw);          //Set Gateway
    esp_netif_str_to_ip4(ETHERNET_SUBNET_MASK, &ip_info.netmask);    //Set Subnet Mask

    esp_netif_set_ip_info(eth_netif, &ip_info);
    
    #endif //ETHERNET_USE_STATIC_IP
    

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
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());


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

#ifdef ETHERNET_USE_STATIC_IP
    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ip_info;

    esp_netif_str_to_ip4(ETHERNET_IP_ADDR, &ip_info.ip);          //Set IP address
    esp_netif_str_to_ip4(ETHERNET_GATEWAY, &ip_info.gw);          //Set Gateway
    esp_netif_str_to_ip4(ETHERNET_SUBNET_MASK, &ip_info.netmask);    //Set Subnet Mask

    esp_netif_set_ip_info(eth_netif, &ip_info);

#endif //ETHERNET_USE_STATIC_IP

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
    ESP_ERROR_CHECK(spi_bus_initialize(ETHERNET_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

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
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi));

    /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
    02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    }));

    // attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle_spi)));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi));
#endif //ETHERNET_USE_W5500
}

