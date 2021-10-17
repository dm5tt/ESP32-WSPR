/*
 * WSPR transmitter
 * ESP32 + Si5351
 *
 * Highly experimental. Some bits of Espressif example code
 * are contained.
 *
 */

/*

Copyright (c) 2018 Dana H. Myers, K6JQ
Copyright (c) 2021 Holger Adams, DM5TT

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/sntp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "driver/i2c.h"

#include "si5351.h"
#include "wspr.h"

static void periodic_timer_callback(void* arg);

static const char *TAG = "main";

/**
 * Pin assignment:
 *
 * Connect Si5351A:
 *      SCL to ESP32 GPIO19
 *      SDA to ESP32 GPIO18
 */

#define I2C_EXAMPLE_MASTER_SCL_IO	22
#define I2C_EXAMPLE_MASTER_SDA_IO	21

/*
 * Set the WiFi info
 */
#define EXAMPLE_WIFI_SSID "es-router"
#define EXAMPLE_WIFI_PASS "sicher73728esslingen!!"

/*
 * FreeRTOS event group to signal when we are connected & ready to
 * make a request
 */
static EventGroupHandle_t s_wifi_event_group;

/*
 * The event group allows multiple bits for each event, but we
 * only care about one event - are we connected to the AP with
 * an IP?
 */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define EXAMPLE_ESP_MAXIMUM_RETRY  100

#define TIME_RETRY_COUNT    10

static int s_retry_num = 0;

static void obtain_time(void);
static void initialize_sntp(void);
static void initialise_wifi(void);
static void event_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data);
/*
 *
 */
static void
obtain_time(void)
{
    int retry;
    time_t now = 0;
    struct tm timeinfo = { 0 };


    initialise_wifi();

    /*
     * Wait for WiFi connected event
     */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    initialize_sntp();

    // wait for time to be set
    retry = TIME_RETRY_COUNT;
    while(timeinfo.tm_year < (2016 - 1900) && (retry-- > 0)) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d)", retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    // Leave the WiFi running so SNTP can poll
}

/*
 *
 */
static void
initialize_sntp(void)
{

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

/*
 *
 */
static void
initialise_wifi(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    ESP_ERROR_CHECK(esp_netif_init());

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/*
 *
 */
static void
event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief i2c master initialization
 */
static esp_err_t
i2c_master_init()
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_flags = 0;
    conf.master.clk_speed = 100000;

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

    return (ESP_OK);
}

/*
 *
 */
int
si5351_write_xfer(uint8_t reg, uint8_t *data, int count)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI5351_BUS_BASE_ADDR << 1, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, count, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return (ret);
}

int
si5351_read_xfer(uint8_t reg, uint8_t *data, int count)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
      (SI5351_BUS_BASE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
	return (ret);
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
      (SI5351_BUS_BASE_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, count, 2);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret);
}

void
si5351_write_byte(uint8_t reg, uint8_t val)
{
	si5351_write_xfer(reg, &val, 1);
}

uint8_t
si5351_read_byte(uint8_t reg)
{
	uint8_t data = 0;

	(void) si5351_read_xfer(reg, &data, 1);
	return (data);
}

void
si5351_start(enum si5351_clock clk, int64_t freq)
{

	(void) si5351_set_freq(freq, clk);
	si5351_output_enable(clk, 0);
	si5351_drive_strength(clk, SI5351_DRIVE_8MA);
}

/*
 *
 */
static uint64_t txFreq = (14095600 + 1500 - 50) * 100;
//static uint64_t txFreq = (10138700 + 1500 - 4) * 100;
//static uint64_t txFreq = (7038600 + 1500 - 4) * 100;
static const uint64_t txStep = (uint64_t)((12000 * 100) / 8192);

static uint8_t syms[162];
static esp_timer_handle_t periodic_timer;
static TaskHandle_t wsprTaskHandle;


static void
periodic_timer_callback(void* arg)
{

    /* kick the transmitter task */
    xTaskNotifyGive(wsprTaskHandle);
}

static void
wsprTransmitter(void *arg)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "baud"
    };
    int txIndex;
    time_t now;
    struct tm timeinfo;

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* init I2C port 0 */
    i2c_master_init();

    si5351_init(SI5351_CRYSTAL_LOAD_10PF, 27000000, 175310);
    si5351_start(SI5351_CLK0, txFreq);

    (void) get_wspr_channel_symbols("<K6JQ> CM88WE 10", syms);

    /* send frame */
    while (true) {
        txIndex = 0;

        /*
         * XXX: do limited duty-cycle here
         */
        do {
            taskYIELD();
            time(&now);
            gmtime_r(&now, &timeinfo);
        } while ((timeinfo.tm_sec != 0) || ((timeinfo.tm_min % 2) != 0));
        printf("start: %d:%d\n", timeinfo.tm_min, timeinfo.tm_sec);

        /* delat 1 second into the frame */
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /* start first baud */
        si5351_set_freq(syms[txIndex++] * txStep + txFreq, SI5351_CLK0);
        si5351_output_enable(SI5351_CLK0, 1);

        /*  start baud timer */
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer,
          (1000000ULL * 8192) / 12000 ));

        /* bang out the rest of the frame under timer control */
        while (txIndex < 162) {
            /* wait for a baud clock */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            si5351_set_freq(syms[txIndex++] * txStep + txFreq, SI5351_CLK0);
        }

        /* turn off the TX */
        si5351_output_enable(SI5351_CLK0, 0);

        /* stop the baud timer */
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    }

    /*
     * we never get here
     */
}

void app_main(void) {
	s_wifi_event_group = xEventGroupCreate();
	time_t now;
	struct tm timeinfo;

	/* get the current time */
	time(&now);
	gmtime_r(&now, &timeinfo);

	// Is time set? If not, tm_year will be (1970 - 1900).
	if (timeinfo.tm_year < (2016 - 1900)) {
		obtain_time();
	}

	//start WSPR transmitter task
	// XXX: check priority, use proper define
	xTaskCreate(wsprTransmitter, "WSPR", 2048, NULL, 10, &wsprTaskHandle);
}
