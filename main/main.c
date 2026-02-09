#include "esp_log.h"
#include "bsp/esp-bsp.h"
// #include "app_disp_fs.h"
#include "driver/gpio.h"
#include <pzem-driver.h>
#include "lvgl.h"
#include "esp_http_server.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"


#define TXPIN 43
#define RXPIN 44

#define ADDRESS 0x10
#define MUXA GPIO_NUM_11
#define MUXB GPIO_NUM_12
#define RELAY_BATT 9
static const char *TAG = "MAIN";
SemaphoreHandle_t lvgl_mutex = NULL;
struct power{
    float sectorA;
    float sectorB;
    float grid;
    float renewable;
    float limit; 

}powerstruct;

QueueHandle_t pzem_queue;


void relay_handle(int sec_no, int dir)
{
    
    gpio_set_direction(GPIO_NUM_40, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_41, GPIO_MODE_OUTPUT);
    switch (sec_no)
    {
    case 0:
        gpio_set_level(GPIO_NUM_40, dir);
        break;
    case 1:
        gpio_set_level(GPIO_NUM_41, dir);
        break;
    default:
        break;
    }

}

void battery_handle(bool dir){
    gpio_set_direction(RELAY_BATT, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_BATT, dir);
}

void power_sample()
    {
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    

    gpio_set_level(GPIO_NUM_11, 0);
    gpio_set_level(GPIO_NUM_12, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    updateValues(0XF8);
    powerstruct.grid = getPower();
    ESP_LOGI(TAG, "grid= %f", powerstruct.grid);
    ESP_LOGI(TAG, "energy1 = %f", getEnergy());
    ESP_LOGI(TAG, "------");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_11, 1);
    gpio_set_level(GPIO_NUM_12, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    updateValues(0XF8);
    powerstruct.sectorB = getPower();
    ESP_LOGI(TAG, "sectorB = %f", powerstruct.sectorB);

    ESP_LOGI(TAG, "------");
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_11, 0);
    gpio_set_level(GPIO_NUM_12, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    updateValues(0XF8);
    powerstruct.renewable = getPower();
    ESP_LOGI(TAG, "renewable = %f", powerstruct.renewable);

    ESP_LOGI(TAG, "------");
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_11, 1);
    gpio_set_level(GPIO_NUM_12, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    updateValues(0XF8);
    powerstruct.sectorA = getPower();
    ESP_LOGI(TAG, "sectorA = %f", powerstruct.sectorA);
}
void dec_algo(){
    powerstruct.limit = 12.0;
    ESP_LOGI(TAG, "ADDITION: %f", powerstruct.sectorA+powerstruct.sectorB);
    if(powerstruct.sectorA+powerstruct.sectorB < powerstruct.renewable){
            relay_handle(0, 1);
        relay_handle(1, 1);
        
    }else if(powerstruct.sectorA<powerstruct.limit && powerstruct.sectorB<powerstruct.limit){
        if(powerstruct.sectorA>powerstruct.sectorB){
            relay_handle(0,1);
            relay_handle(1,0);
        }else{
            relay_handle(0,0);
            relay_handle(1,1);
        }
    }else if(powerstruct.sectorA<powerstruct.renewable){
        relay_handle(0,1);

    }else if(powerstruct.sectorB<powerstruct.renewable){
        relay_handle(1,1);
    }else{
        relay_handle(0,0);
        relay_handle(1,0);
    }
}

void display_send(){
    unsigned char  data_power_renewable[8]={0x5A, 0xA5, 0x05, 0x82, 0x52, 0x00, 0x01, 0x12};
    int a = powerstruct.grid;
    uint8_t high_byte = (a>> 8) & 0xFF;
    uint8_t low_byte  = a & 0xFF;
    data_power_renewable[6] = high_byte;
    data_power_renewable[7] = low_byte;
   // ESP_LOGE(TAG, "%i", uart_write_bytes(UART_NUM_2, data_power_renewable, 8));


}
void main_task(void *arg){
    while(1){
        power_sample();
            dec_algo();
        xQueueOverwrite(pzem_queue, &powerstruct);
        // relay_handle(1,1)
        display_send();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}



void app_main(void)
{

    bsp_i2c_init();
    initialize_pzem(TXPIN, RXPIN);
    relay_handle(0,1);
    relay_handle(1,1);
    lvgl_mutex = xSemaphoreCreateMutex();

    uart_config_t uart_config;
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk = UART_SCLK_APB;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 256, 0, 0, NULL,0 ));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_21, GPIO_NUM_13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    powerstruct.limit = 8.0;

    pzem_queue = xQueueCreate(1, sizeof(powerstruct));
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
        .double_buffer = 0,
        .flags = {
            .buff_dma = true,
        }
    };
    bsp_display_start_with_config(&cfg);
    
    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    xTaskCreate(main_task, "main-task", 4096, NULL, 5, NULL);
    



    
}