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

void setpixel(uint16_t LedN)
{

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


static void matrixUpdate(void *pvParameter)
{

    uint32_t red = 25;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

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

    

uint8_t ottenok=50;
uint8_t yarkost=7;
uint8_t nextPixel=0;
uint8_t j=0;

while(true){
	 // Build RGB values
                hue = 2* 360 / CONFIG_EXAMPLE_STRIP_LED_NUMBER + start_rgb;
                led_strip_hsv2rgb(hue, ottenok,yarkost, &red, &green, &blue);

               // ESP_ERROR_CHECK(strip->set_pixel(strip, 1, red, green, blue));
           
                //ESP_ERROR_CHECK(strip->set_pixel(strip, i * 16 + j, 0, 0, 0));
 ESP_ERROR_CHECK(strip->refresh(strip, 256));
                for(int i=0;i<48;i++){
                    
                    //1
                    nextPixel=one_chet[i]*Mas[j][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                    //2
                    nextPixel=two_chet[i]*Mas[j][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                     //3
                    nextPixel=thr_chet[i]*Mas[j][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                     //4
                    nextPixel=for_chet[i]*Mas[j][i];
                    if(nextPixel){
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel , red, green, blue));
                    }else{
                    ESP_ERROR_CHECK(strip->set_pixel(strip, nextPixel, 0, 0, 0));
                    }

                    //razdel

                     nextPixel=razdel[j];
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
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
          ///  strip->clear(strip, 50);
           vTaskDelay(60000/ portTICK_PERIOD_MS);  
ESP_ERROR_CHECK(strip->clear(strip, 256));  
j++;
start_rgb =start_rgb+100;

if(j==10) j=0;
    }
}    




void app_main(void)
{
  

    xTaskCreate(&matrixUpdate, "updateMatrix", 2096, NULL, 5, NULL);
    
    while (true) {
        
        vTaskDelay(1000/ portTICK_PERIOD_MS); 
               
 
    }
}
