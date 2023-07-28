/* RMT example -- RGB LED Strip

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "example";
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define EXAMPLE_CHASE_SPEED_MS (100)

//////////////////////////////////

#include <string.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/apps/sntp.h"

#include "shtc3.h"
#include "als303.h"

#include "esp_http_client.h"



char api_key[] = "WR3QEDM9KWWGC1Z0";



#define SENSOR_ADDR_SHT 0x70
#define SENSOR_ADDR_ALS 0x29


#define FLAG_TIME 0
#define FLAG_SHT_SENSOR 1
#define FLAG_ALS_SENSOR 2
uint8_t FLAG_NOW=1;
char SHT_DATA[10];
uint8_t ALS_DATA[10];

float temperature =0, humidity =0; 
float light=0;




static void time_init(void);

static void obtain_time(void);
static void initialize_sntp(void);
static void reboot(char *msg_err); //called only by main thread

/* Only used in startup: if time_init() can't set current time for the first time -> reboot() */
static bool ONCE = true;



/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "KARATAS"
#define EXAMPLE_ESP_WIFI_PASS      "2208mefta"
#define EXAMPLE_ESP_MAXIMUM_RETRY  30

///
#define THINGSPEAK_API_KEY "YOUR_KEY"

// HTTP event handler
esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

void http_post_task(void *pvParameters) {
    char post_data[256];
    while (1) {

        sprintf(post_data, "api_key=%s&field1=%.2f&field2=%.2f&field3=%0.2f",
                THINGSPEAK_API_KEY, temperature, humidity, light);

        esp_http_client_config_t config = {
            .url = "https://api.thingspeak.com/update",
            .event_handler = _http_event_handler,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_post_field(client, post_data, strlen(post_data));

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);

        vTaskDelay(pdMS_TO_TICKS(60000)); // Пауза в 10 секунд перед следующей отправкой
    }
}


///

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
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


//system
static void reboot(char *msg_err)
{
	int i;

	ESP_LOGE(TAG, "%s", msg_err);
    for(i=3; i>=0; i--){
        ESP_LOGW(TAG, "Restarting in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGW(TAG, "Restarting now");
    fflush(stdout);

    esp_restart();
}


///time
static void time_init()
{
	time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

	ESP_LOGI(TAG, "1Connecting to WiFi and getting time over NTP.");
	obtain_time();
	time(&now);  //update 'now' variable with current time

    //setting timezone to Greenwich
    setenv("TZ", "MSK-3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "TIME INFO: The Moscow date/time is: %s", strftime_buf);

}

static void obtain_time()
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;

    initialize_sntp();

    //wait for time to be set
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if(retry >= retry_count){ //can't set time
    	if(ONCE) //if it is first time -> reboot: no reason to sniff with wrong time
    		reboot("No response from server after several time. Impossible to set current time");
    }
    else{
    	ONCE = false;
    }
}

static void initialize_sntp()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL); //automatically request time after 1h
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}



////

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

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
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
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
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}





void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


int Mas[10][48] = {
    {0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,1,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,1,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0},
    {0,1,1,1,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0},
    {1,1,1,1,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0},
    {0,0,0,1,0,0,0,0,1,1,0,0,0,1,0,1,0,0,1,0,0,1,0,0,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {1,1,1,1,1,0,1,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0},
    {0,0,1,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0},
	{1,1,1,1,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0},
	{0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,1,1,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,0,0,0,0}
};
    
int one_chet[48]={2,3,4,5,6,7,18,19,20,21,22,23,34,35,36,37,38,39,50,51,52,53,54,55,66,67,68,69,70,71,82,83,84,85,86,87,98,99,100,101,102,103,114,115,116,117,118,119};
int two_chet[48]={10,11,12,13,14,15,26,27,28,29,30,31,42,43,44,45,46,47,58,59,60,61,62,63,74,75,76,77,78,79,90,91,92,93,94,95,106,107,108,109,110,111,122,123,124,125,126,127};
int thr_chet[48]={130,131,132,133,134,135,146,147,148,149,150,151,162,163,164,165,166,167,178,179,180,181,182,183,194,195,196,197,198,199,210,211,212,213,214,215,226,227,228,229,230,231,242,243,244,245,246,247};
int for_chet[48]={138,139,140,141,142,143,154,155,156,157,158,159,170,171,172,173,174,175,186,187,188,189,190,191,202,203,204,205,206,207,218,219,220,221,222,223,234,235,236,237,238,239,250,251,252,253,254,255};

//razdelitel
uint8_t razdel[10]={56,72,88,104,120,136,152,168,184,200};

static void light_measure(void *pvParameter)
{


  init_sensor_als(I2C_MODE_MASTER, 33, 14, I2C_MASTER_FREQ_100KHZ);
  start_get_data(SENSOR_ADDR_ALS);
  setup(SENSOR_ADDR_ALS);
  uint8_t data_av=0;

   while (1)
    {
        //get_ID_sensor(SENSOR_ADDR);
       // ControlRegisters(SENSOR_ADDR);
       // ALS_Measurement_Rates(SENSOR_ADDR);

   
   status(SENSOR_ADDR_ALS,&data_av);
   printf("data:%x\n",data_av);
   light=0;
   if(data_av==4){
   get_data(SENSOR_ADDR_ALS,&light);
   }
    //read_out(SENSOR_ADDR);

      //  wakeup_sensor(SENSOR_ADDR);
      
        //read_out(SENSOR_ADDR, T_FIRST_N, &temperature, &humidity);
       // sleep_sensor(SENSOR_ADDR);
        ESP_LOGI(TAG, "Light: %f", light);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

static void temp_hum_measure(void *pvParameter)
{
 init_sensor(I2C_MODE_MASTER, 25, 26, I2C_MASTER_FREQ_100KHZ);

    while (1)
    {
        //get_ID_sensor(SENSOR_ADDR);

        wakeup_sensor(SENSOR_ADDR_SHT);
       
        read_out(SENSOR_ADDR_SHT, T_FIRST_N, &temperature, &humidity);
        sleep_sensor(SENSOR_ADDR_SHT);
        ESP_LOGI(TAG, "Temperature: %0.2f, Humidade: %0.2f", temperature, humidity);
      sprintf(SHT_DATA,"%0.2f:%0.2f",temperature,humidity);
        printf("DATA_SHT: %s\n",SHT_DATA);
      //  printf()
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
}


static void matrixUpdate(void *pvParameter)
{

    uint32_t red = 25;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    //time
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    uint8_t H1=0;
    uint8_t H2=0;
    uint8_t M1=0;
    uint8_t M2=0;

    //
    bool ind=false;


    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");

    
if(FLAG_NOW==FLAG_TIME){
    
}
uint8_t ottenok=50;
uint8_t yarkost=7;
uint8_t nextPixel=0;


while(true){
// Build RGB values
 hue = 2* 360 / CONFIG_EXAMPLE_STRIP_LED_NUMBER + start_rgb;
led_strip_hsv2rgb(hue, ottenok,yarkost, &red, &green, &blue);

if(FLAG_NOW==FLAG_TIME)
{
time(&now);
localtime_r(&now, &timeinfo);
strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
ESP_LOGI(TAG, "TIME INFO: The Moscow date/time is: %s", strftime_buf);
//printf("H1:%d\n",strftime_buf[11]-48);
//printf("H2:%d\n",strftime_buf[12]-48);
//printf("M1:%d\n",strftime_buf[14]-48);
//printf("M2:%d\n",strftime_buf[15]-48);
ind=!ind;
//printf("Ind:%d\n",ind);
//set time
H1=strftime_buf[11]-48;
H2=strftime_buf[12]-48;
M1=strftime_buf[14]-48;
M2=strftime_buf[15]-48;
if(ind){
    ESP_ERROR_CHECK(strip->set_pixel(strip, 119 , red, green, blue)); 
    ESP_ERROR_CHECK(strip->set_pixel(strip, 120 , red, green, blue)); 
}
}else if(FLAG_NOW==FLAG_SHT_SENSOR){


ESP_ERROR_CHECK(strip->set_pixel(strip, 32 , red, green, blue)); 
ESP_ERROR_CHECK(strip->set_pixel(strip, 33 , red, green, blue)); 
ESP_ERROR_CHECK(strip->set_pixel(strip, 34 , red, green, blue)); 

ESP_ERROR_CHECK(strip->set_pixel(strip, 49 , red, green, blue));
ESP_ERROR_CHECK(strip->set_pixel(strip, 65 , red, green, blue));
H1=SHT_DATA[0]-48;
H2=SHT_DATA[1]-48;
M1=SHT_DATA[6]-48;
M2=SHT_DATA[7]-48;
printf("H1:%d\n",SHT_DATA[0]-48);
printf("H2:%d\n",SHT_DATA[1]-48);
printf("M1:%d\n",SHT_DATA[3]-48);
printf("M2:%d\n",SHT_DATA[4]-48);


}




 for(int i=0;i<48;i++){
                    
                    //1
                    nextPixel=one_chet[i]*Mas[H1][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                    //2
                    nextPixel=two_chet[i]*Mas[H2][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                     //3
                    nextPixel=thr_chet[i]*Mas[M1][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                     //4
                    nextPixel=for_chet[i]*Mas[M2][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }	

                    
 }
                printf("\n");

 



vTaskDelay(10/ portTICK_PERIOD_MS);  

// Flush RGB values to LEDs
          
            ESP_ERROR_CHECK(strip->refresh(strip, 256));
            ESP_ERROR_CHECK(strip->clear(strip, 256));  
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
          ///  strip->clear(strip, 50);
        
           vTaskDelay(1000/ portTICK_PERIOD_MS);  


//start_rgb =start_rgb+100;


    }
}    





void app_main(void)
{

    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    time_init(); 
 


    xTaskCreate(&matrixUpdate, "updateMatrix", 2096, NULL, 5, NULL);
    xTaskCreate(&temp_hum_measure, "temp_hum", 2096, NULL, 5, NULL);
    xTaskCreate(&light_measure, "light", 4096, NULL, 5, NULL);
    xTaskCreate(&http_post_task, "send_data_to_thingspeak", 8192, NULL, 6, NULL);
    
    while (true) {
        
        FLAG_NOW=! FLAG_NOW;
        vTaskDelay(10000/ portTICK_PERIOD_MS); 
               
 
    }
}
