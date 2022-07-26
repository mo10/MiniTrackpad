#include "hardware.h"
#include "config.h"

#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define TAG "Hardware"

static void backlight_init();
static void lcd_im_init();

void hardware_init(){
    backlight_init();
    lcd_im_init();
}

static void backlight_init() {
    ESP_LOGI(TAG, "Initializing backlight...");

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HW_LCD_BKL,
        .duty           = 255,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void lcd_im_init(){
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (
            (1ULL<<HW_LCD_IM1)|
            (1ULL<<HW_LCD_IM2)|
            /* 8080 */
            (1ULL<<HW_LCD_RDX)|
            (1ULL<<HW_LCD_DB0)|
            (1ULL<<HW_LCD_DB1)|
            (1ULL<<HW_LCD_DB2)|
            (1ULL<<HW_LCD_DB3)|
            (1ULL<<HW_LCD_DB4)|
            (1ULL<<HW_LCD_DB5)|
            (1ULL<<HW_LCD_DB6)|
            (1ULL<<HW_LCD_DB7)
            ),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_set_level(HW_LCD_RDX, 0);
    gpio_set_level(HW_LCD_DB0, 0);
    gpio_set_level(HW_LCD_DB1, 0);
    gpio_set_level(HW_LCD_DB2, 0);
    gpio_set_level(HW_LCD_DB3, 0);
    gpio_set_level(HW_LCD_DB4, 0);
    gpio_set_level(HW_LCD_DB5, 0);
    gpio_set_level(HW_LCD_DB6, 0);
    gpio_set_level(HW_LCD_DB7, 0);
    
    #ifdef HW_LCD_USE_3SPI
        gpio_set_level(HW_LCD_IM1, 0);
        gpio_set_level(HW_LCD_IM2, 1);
    #endif
    #ifdef HW_LCD_USE_4SPI
        gpio_set_level(HW_LCD_IM1, 1);
        gpio_set_level(HW_LCD_IM2, 1);
    #endif
    #ifdef HW_LCD_USE_8080
        gpio_set_level(HW_LCD_IM1, 1);
        gpio_set_level(HW_LCD_IM2, 0);
    #endif
}