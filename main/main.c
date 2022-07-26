#include <stdio.h>

#include "hardware.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_freertos_hooks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#define LV_TICK_PERIOD_MS 1


#define TAG "Main"

static void lvgl_task(void *pvParameter);
static void lvgl_tick_inc(void *arg);

void app_main(void)
{
    ESP_LOGI(TAG,"Hello world");
    hardware_init();
    xTaskCreatePinnedToCore(lvgl_task, "gui", 4096*2, NULL, 0, NULL, 1);
}

static void lvgl_task(void *pvParameter){
    ESP_LOGI(TAG, "Initializing LVGL...");

    lv_init();

    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE*sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE*sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = 480;
    disp_drv.ver_res = 320;
    lv_disp_drv_register(&disp_drv);

    // lv_indev_drv_t indev_drv;
    // lv_indev_drv_init(&indev_drv);
    // indev_drv.type = LV_INDEV_TYPE_POINTER;
    // indev_drv.read_cb = gt911_read;
    // // Touch 
    // lv_indev_t * mouse_indev =lv_indev_drv_register(&indev_drv);
    // if (!mouse_indev) {
    //     ESP_LOGI(TAG,"\n lv_indev_drv_register failed");
    // }
    // lv_obj_t * cursor_obj =  lv_img_create(lv_scr_act()); /*Create an image object for the cursor */
    // lv_img_set_src(cursor_obj, LV_SYMBOL_PLUS);             /*Set the image source*/
    // lv_indev_set_cursor(mouse_indev, cursor_obj);               /*Connect the image  object to the driver*/

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lvgl_tick_inc,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /*Create a label with the new style*/
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");

    lv_obj_t * spinner = lv_spinner_create(lv_scr_act(), 1000, 60);
	lv_obj_set_pos(spinner, 190, 60);
	lv_obj_set_size(spinner, 100, 100);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
    }
    ESP_LOGI(TAG, "LVGL Stopped");
}

static void lvgl_tick_inc(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
