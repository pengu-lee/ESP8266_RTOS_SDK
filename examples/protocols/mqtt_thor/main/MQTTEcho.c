/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "MQTTClient.h"

#include "MCP39F511N.h"
#include "driver/uart.h"
#include "driver/gpio.h"


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "MHLee"
#define EXAMPLE_WIFI_PASS "0975626921"
// #define EXAMPLE_WIFI_SSID "ThorAP"
// #define EXAMPLE_WIFI_PASS "0975626921"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

#define MQTT_BROKER  "mqtt.mcs.mediatek.com"  /* MQTT Broker Address*/
#define MQTT_PORT    1883             /* MQTT Port*/

#define MQTT_CLIENT_THREAD_NAME         "mqtt_client_thread"
#define MQTT_CLIENT_THREAD_STACK_WORDS  8192
#define MQTT_CLIENT_THREAD_PRIO         8

#define BUF_SIZE (1024)

#define GPIO_OUTPUT_IO_0    13  // Relay On pin
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))
#define GPIO_INPUT_IO_0     12  // Relay Off pin
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))

static const char *TAG = "example";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}
static void initialise_gpio(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO13
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    //bit mask of the pins that you want to set,e.g.GPIO12
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void messageArrived(MessageData* data)
{
    printf("Message arrived: %s\n", (char*)data->message->payload);
    uint8_t* listenStr = data->message->payload;
    printf("listenStr: %s\n", listenStr);

    char* pch = strstr ((const char*)listenStr,"switch01,");
    if(pch[9] == 0x30)
        gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    else if(pch[9] == 0x31)
        gpio_set_level(GPIO_OUTPUT_IO_0, 1);
}

static void mqtt_client_thread(void* pvParameters)
{
    MQTTClient client;
    Network network;
    unsigned char sendbuf[80], readbuf[80] = {0};
    int rc = 0;
    unsigned int count = 0;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    printf("mqtt client thread starts\n");

    /* Wait for the callback to set the CONNECTED_BIT in the
       event group.
    */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");

    NetworkInit(&network);
    MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

    char* address = MQTT_BROKER;

    if ((rc = NetworkConnect(&network, address, MQTT_PORT)) != 0) {
        printf("Return code from network connect is %d\n", rc);
    }

#if defined(MQTT_TASK)

    if ((rc = MQTTStartTask(&client)) != pdPASS) {
        printf("Return code from start tasks is %d\n", rc);
    } else {
        printf("Use MQTTStartTask\n");
    }

#endif

    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = "ESP8266_thor";

    if ((rc = MQTTConnect(&client, &connectData)) != 0) {
        printf("Return code from MQTT connect is %d\n", rc);
    } else {
        printf("MQTT Connected\n");
    }

    if ((rc = MQTTSubscribe(&client, "mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/switch01", 2, messageArrived)) != 0) {
        printf("Return code from MQTT subscribe is %d\n", rc);
    } else {
        printf("MQTT subscribe to topic \"mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/switch01\"\n");
    }


    while (++count) {
        MQTTMessage message;
        char payload[30];
        // Configure a temporary buffer for the incoming data
        uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
        float vrms = 0.0, freq = 0.0, irms1 = 0.0, watt1 = 0.0;
        uint64_t energy1 = 0;

        readPower();
        vrms = getVolts();
        freq = getFrequency();
        irms1 = getAmps1();
        watt1 = getWatts1();
        energy1 = getImportEnergy1();

        printf("Import Energy1: %llu\n", energy1);

        /*message.qos = QOS1;
        message.retained = 0;
        message.payload = payload;
        sprintf(payload, ",V1,%0.2f", vrms);
        // sprintf(payload, ",led,0");
        message.payloadlen = strlen(payload);

        if ((rc = MQTTPublish(&client, "mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/V1", &message)) != 0) {
            printf("Return code from MQTT publish is %d\n", rc);
        } else {
            printf("MQTT publish topic \"mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/V1\", message number is %0.2f\n", vrms);
        }
        vTaskDelay(500 / portTICK_RATE_MS);

        sprintf(payload, ",I1,%0.2f", irms1);
        message.payloadlen = strlen(payload);
        if ((rc = MQTTPublish(&client, "mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/I1", &message)) != 0) {
            printf("Return code from MQTT publish is %d\n", rc);
        } else {
            printf("MQTT publish topic \"mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/I1\", message number is %0.2f\n", irms1);
        }
        vTaskDelay(500 / portTICK_RATE_MS);

        sprintf(payload, ",P1,%0.2f", watt1);
        message.payloadlen = strlen(payload);
        if ((rc = MQTTPublish(&client, "mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/P1", &message)) != 0) {
            printf("Return code from MQTT publish is %d\n", rc);
        } else {
            printf("MQTT publish topic \"mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/P1\", message number is %0.2f\n", watt1);
        }
        vTaskDelay(100 / portTICK_RATE_MS);

        sprintf(payload, ",led,%d", gpio_get_level(GPIO_INPUT_IO_0));
        message.payloadlen = strlen(payload);
        if ((rc = MQTTPublish(&client, "mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/led", &message)) != 0) {
            printf("Return code from MQTT publish is %d\n", rc);
        } else {
            printf("MQTT publish topic \"mcs/DzdZiwp0/Tp1pIcId3xSlzl5o/led\", message number is %d\n", gpio_get_level(GPIO_INPUT_IO_0));
        }*/

        free(data);
        vTaskDelay(2000 / portTICK_RATE_MS);  //send every 1 seconds
    }

    printf("mqtt_client_thread going to be deleted\n");
    vTaskDelete(NULL);
    return;
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    initialise_gpio();
    mcp_init();
    xTaskCreate(&mqtt_client_thread,
                MQTT_CLIENT_THREAD_NAME,
                MQTT_CLIENT_THREAD_STACK_WORDS,
                NULL,
                MQTT_CLIENT_THREAD_PRIO,
                NULL);
}
