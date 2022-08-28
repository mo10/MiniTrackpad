#include "gt911_hal.h"
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#define TAG "gt911_hal"

volatile static gt911_instance_t instance = {
    .config = NULL,
    .is_initialized = 0,
    .i2c_addr = 0xba >> 1,
};

static SemaphoreHandle_t xSemaphore = NULL;
static uint8_t gt911_irq = 0;

void print_gt911_config_t(gt911_config_t *ptr);

static inline void hal_delay(uint32_t v){
    vTaskDelay(pdMS_TO_TICKS(v));
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    if(instance.is_initialized)
        xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static void gt911_isr_task(void* arg)
{
  for(;;) {
    if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE && instance.is_initialized) {
        gt911_read_points(&instance.info);
    }
  }
}

static esp_err_t hal_gpio_init(){
    esp_err_t err;

    gpio_pad_select_gpio(instance.config->rst_pin);
    gpio_set_direction(instance.config->rst_pin, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(instance.config->int_pin);
    gpio_set_direction(instance.config->int_pin, GPIO_MODE_OUTPUT);

    gpio_set_level(instance.config->rst_pin, 0);
    gpio_set_level(instance.config->int_pin, 0);
    hal_delay(11);
    
    gpio_set_level(instance.config->rst_pin, 1);
    hal_delay(6);
    
    gpio_pad_select_gpio(instance.config->int_pin);
    gpio_set_direction(instance.config->int_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(instance.config->int_pin, GPIO_FLOATING);
    gpio_set_intr_type(instance.config->int_pin, GPIO_INTR_POSEDGE);

    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(gt911_isr_task, "gt911_isr_task", 2048, NULL, 10, NULL);

    err = gpio_install_isr_service(0);
    if(err) goto Failed;
    err = gpio_isr_handler_add(instance.config->int_pin, gpio_isr_handler, NULL);
    if(err) goto Failed;

    return err;
Failed:
    ESP_LOGE(TAG,"Failed to initialize gt911 gpio, err:%d", err);

    return err;
}

static inline esp_err_t hal_i2c_read(uint16_t reg, uint8_t *data, size_t len){
    esp_err_t err;
    uint16_t swap_reg = ((reg & 0xff) << 8) | ((reg & 0xff00) >> 8);

    err = i2c_master_write_read_device(instance.config->i2c_port, instance.i2c_addr, (uint8_t *)&swap_reg, sizeof(swap_reg), data, len, 2000 / portTICK_PERIOD_MS);
    if(err) { 
        ESP_LOGI(TAG, "I2C read2 failed, err:%d", err);
        return err;
    }
    return err;
}

static inline esp_err_t hal_i2c_write(uint16_t reg, uint8_t *data, size_t len){
    esp_err_t err = ESP_OK;
    uint16_t swap_reg = ((reg & 0xff) << 8) | ((reg & 0xff00) >> 8);

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, instance.i2c_addr << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write(handle, (uint8_t *)&swap_reg, sizeof(swap_reg), true);
    if (err != ESP_OK) {
        goto end;
    }
    err = i2c_master_write(handle, data, len, true);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(instance.config->i2c_port, handle,  2000 / portTICK_PERIOD_MS);

end:
    i2c_cmd_link_delete(handle);
    if(err)
        ESP_LOGI(TAG, "I2C write failed, err:%d", err);
    return err;
}

static esp_err_t hal_i2c_init(){
    esp_err_t err;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = instance.config->i2c_sda_pin,
        .scl_io_num = instance.config->i2c_scl_pin,
        .sda_pullup_en = 1,
        .scl_pullup_en = 1,
        .master.clk_speed = instance.config->i2c_speed,
        .clk_flags = 0,
    };
    err = i2c_param_config(instance.config->i2c_port, &conf);
    if(err) goto Failed;
    err = i2c_driver_install(instance.config->i2c_port, conf.mode, 0, 0, 0);
    if(err) goto Failed;
    return err;
Failed:
    ESP_LOGI(TAG,"Failed to initialize gt911 i2c, err:%d", err);

    return err;
}

int gt911_init(gt911_init_config_t *config) {
    int err = 0;
    if(instance.is_initialized)
        goto Failed;

    ESP_LOGI(TAG,"GT911 start init");

    instance.config = config;

    if((err = hal_gpio_init()))
        goto Failed;

    hal_delay(100);

    if((err = hal_i2c_init()))
        goto Failed;
    
    /* Check & update config*/
    gt911_send_command(GT911_SOFT_RESET);
    gt911_read_config(&instance.cfg);
    gt911_overwrite_config(&instance.cfg);
    gt911_send_command(GT911_READ_COORDINATE);

    gt911_read_info(&instance.info);
    ESP_LOGI(TAG,"Product:%s", instance.info.product_id);

    if(strcmp(instance.info.product_id, "911"))
        goto Failed;

    instance.is_initialized = 1;
 
    // hal_hex_dump(&instance.cfg, sizeof(instance.cfg));
    // print_gt911_config_t(&instance.cfg);
    return 0;
Failed:
    ESP_LOGE(TAG, "Failed to initialize gt911, err:%d",err);

    return -1;
}

inline uint8_t gt911_isready() {
    return instance.is_initialized;
}

inline int gt911_read_config(gt911_config_t *config) {
    return hal_i2c_read(GT911_REG_CONFIG_BASE, (uint8_t *)config, sizeof(gt911_config_t));
}

inline int gt911_write_config(gt911_config_t *config) {
    return hal_i2c_write(GT911_REG_CONFIG_BASE, (uint8_t *)config, sizeof(gt911_config_t));
}

int gt911_overwrite_config(gt911_config_t *config) {
    uint8_t need_update = 0;
    #define check_update(member, value) \
        if(member != value){ \
            member = value; \
            need_update = 1; \
        }
    
    check_update(config->x_output_max, 480);
    check_update(config->y_output_max, 320);
    check_update(config->touch_number.value, 5);
    check_update(config->module_switch1.int_mode, 0x00);
    check_update(config->module_switch1.x2y_mode, 0x01);
    check_update(config->module_switch1.sensor_reseral, 0x01); /* X2X */
    check_update(config->module_switch1.driver_reseral, 0x00); /* Y2Y */
    check_update(config->refresh_rate.report_rate, 0x00); /* 5ms */
    if(need_update){
        ESP_LOGI(TAG, "Overwrite config");

        config->config_version = 0x00;
        config->config_checksum = gt911_calc_checksum(config);
        config->config_fresh = 1;

        return gt911_write_config(config);
    }
    return 0;
}

inline int gt911_read_info(gt911_coordinate_info_t *info) {
    return hal_i2c_read(GT911_REG_PRODUCT_INFO, (uint8_t *)info, sizeof(gt911_coordinate_info_t));
}

int gt911_read_points(gt911_coordinate_info_t *info) {
    if(hal_i2c_read(GT911_REG_POINT_INFO, (uint8_t *)&info->info, 1))
        goto Failed;

    if(info->info.buffer_status && info->info.total_points > 0) {
        /* points ready */
        //memset(info->points, 0, sizeof(info->points));
        if(hal_i2c_read(GT911_REG_POINT_BASE, (uint8_t *)info->points, sizeof(info->points[0]) * info->info.total_points)){
            goto Failed;
        }
    }

    /* clean buffer status flag */
    uint8_t value = 0;
    return hal_i2c_write(GT911_REG_POINT_INFO, &value, 1);

Failed:
    ESP_LOGE(TAG, "read points failed");
    return -1;

}

inline int gt911_send_command(gt911_command_t command) {
    return hal_i2c_write(GT911_REG_COMMAND, &command, 1);
}

uint8_t gt911_calc_checksum(gt911_config_t *config) {
    uint8_t checksum = 0;
    uint8_t i;
    uint8_t *ptr = (uint8_t *) config;

    for (i = 0; i < (sizeof(gt911_config_t) - 2); i++) {
        checksum += (ptr[i] & 0xFF);
    }
    return ~(checksum & 0xFF) + 1;
}



bool gt911_lv_indev_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {

    // if(instance.info.info.total_points == 0){
    //     data->point.x = 0;
    //     data->point.y = 0;
    //     data->state = LV_INDEV_STATE_REL;
    //     return false;
    // }
    data->point.x = instance.info.points[0].x;
    data->point.y = instance.info.points[0].y;
    data->state = LV_INDEV_STATE_PR;

    if(instance.info.info.buffer_status && instance.info.info.total_points == 0){
        data->state = LV_INDEV_STATE_PR;
    }

    return false;
}



/* Debugging */

void print_gt911_config_t(gt911_config_t *ptr) {
    printf("gt911_config_t data = {\n");
    printf("%*c.config_version = '%c', /* 0x%02x */\n", 4, ' ',ptr->config_version, ptr->config_version);
    printf("%*c.x_output_max = 0x%04x, /* %d */\n", 4, ' ',ptr->x_output_max, ptr->x_output_max);
    printf("%*c.y_output_max = 0x%04x, /* %d */\n", 4, ' ',ptr->y_output_max, ptr->y_output_max);
    printf("%*c.touch_number = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->touch_number.value, ptr->touch_number.value);
    printf("%*c.reserved = 0x%02x, /* %d */\n", 8, ' ',ptr->touch_number.reserved, ptr->touch_number.reserved);
    printf("%*c},\n", 4, ' ');
    printf("%*c.module_switch1 = {\n", 4, ' ');
    printf("%*c.int_mode = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.int_mode, ptr->module_switch1.int_mode);
    printf("%*c.sito = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.sito, ptr->module_switch1.sito);
    printf("%*c.x2y_mode = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.x2y_mode, ptr->module_switch1.x2y_mode);
    printf("%*c.stretch_rank = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.stretch_rank, ptr->module_switch1.stretch_rank);
    printf("%*c.sensor_reseral = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.sensor_reseral, ptr->module_switch1.sensor_reseral);
    printf("%*c.driver_reseral = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch1.driver_reseral, ptr->module_switch1.driver_reseral);
    printf("%*c},\n", 4, ' ');
    printf("%*c.module_switch2 = {\n", 4, ' ');
    printf("%*c.touch_key = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.touch_key, ptr->module_switch2.touch_key);
    printf("%*c.hotknot_en = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.hotknot_en, ptr->module_switch2.hotknot_en);
    printf("%*c.approch_en = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.approch_en, ptr->module_switch2.approch_en);
    printf("%*c.reserved1 = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.reserved1, ptr->module_switch2.reserved1);
    printf("%*c.first_filte_dis = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.first_filte_dis, ptr->module_switch2.first_filte_dis);
    printf("%*c.reserved2 = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch2.reserved2, ptr->module_switch2.reserved2);
    printf("%*c},\n", 4, ' ');
    printf("%*c.shake_count = {\n", 4, ' ');
    printf("%*c.pressing = 0x%02x, /* %d */\n", 8, ' ',ptr->shake_count.pressing, ptr->shake_count.pressing);
    printf("%*c.released = 0x%02x, /* %d */\n", 8, ' ',ptr->shake_count.released, ptr->shake_count.released);
    printf("%*c},\n", 4, ' ');
    printf("%*c.filter = {\n", 4, ' ');
    printf("%*c.normal_filter = 0x%02x, /* %d */\n", 8, ' ',ptr->filter.normal_filter, ptr->filter.normal_filter);
    printf("%*c.first_filter = 0x%02x, /* %d */\n", 8, ' ',ptr->filter.first_filter, ptr->filter.first_filter);
    printf("%*c},\n", 4, ' ');
    printf("%*c.large_touch = 0x%02x, /* %d */\n", 4, ' ',ptr->large_touch, ptr->large_touch);
    printf("%*c.noise_reduction = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->noise_reduction.value, ptr->noise_reduction.value);
    printf("%*c.reserved = 0x%02x, /* %d */\n", 8, ' ',ptr->noise_reduction.reserved, ptr->noise_reduction.reserved);
    printf("%*c},\n", 4, ' ');
    printf("%*c.screen_touch_level = 0x%02x, /* %d */\n", 4, ' ',ptr->screen_touch_level, ptr->screen_touch_level);
    printf("%*c.screen_leave_level = 0x%02x, /* %d */\n", 4, ' ',ptr->screen_leave_level, ptr->screen_leave_level);
    printf("%*c.low_power_control = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->low_power_control.value, ptr->low_power_control.value);
    printf("%*c.reserved = 0x%02x, /* %d */\n", 8, ' ',ptr->low_power_control.reserved, ptr->low_power_control.reserved);
    printf("%*c},\n", 4, ' ');
    printf("%*c.refresh_rate = {\n", 4, ' ');
    printf("%*c.report_rate = 0x%02x, /* %d */\n", 8, ' ',ptr->refresh_rate.report_rate, ptr->refresh_rate.report_rate);
    printf("%*c.pluse_width = 0x%02x, /* %d */\n", 8, ' ',ptr->refresh_rate.pluse_width, ptr->refresh_rate.pluse_width);
    printf("%*c},\n", 4, ' ');
    printf("%*c.x_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->x_threshold, ptr->x_threshold);
    printf("%*c.y_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->y_threshold, ptr->y_threshold);
    printf("%*c.x_speed_limit = 0x%02x, /* %d */\n", 4, ' ',ptr->x_speed_limit, ptr->x_speed_limit);
    printf("%*c.y_speed_limit = 0x%02x, /* %d */\n", 4, ' ',ptr->y_speed_limit, ptr->y_speed_limit);
    printf("%*c.space = {\n", 4, ' ');
    printf("%*c.bottom = 0x%02x, /* %d */\n", 8, ' ',ptr->space.bottom, ptr->space.bottom);
    printf("%*c.top = 0x%02x, /* %d */\n", 8, ' ',ptr->space.top, ptr->space.top);
    printf("%*c.right = 0x%02x, /* %d */\n", 8, ' ',ptr->space.right, ptr->space.right);
    printf("%*c.left = 0x%02x, /* %d */\n", 8, ' ',ptr->space.left, ptr->space.left);
    printf("%*c},\n", 4, ' ');
    printf("%*c.mini_filter = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->mini_filter.value, ptr->mini_filter.value);
    printf("%*c.reserved = 0x%02x, /* %d */\n", 8, ' ',ptr->mini_filter.reserved, ptr->mini_filter.reserved);
    printf("%*c},\n", 4, ' ');
    printf("%*c.stretch_r0 = 0x%02x, /* %d */\n", 4, ' ',ptr->stretch_r0, ptr->stretch_r0);
    printf("%*c.stretch_r1 = 0x%02x, /* %d */\n", 4, ' ',ptr->stretch_r1, ptr->stretch_r1);
    printf("%*c.stretch_r2 = 0x%02x, /* %d */\n", 4, ' ',ptr->stretch_r2, ptr->stretch_r2);
    printf("%*c.stretch_rm = 0x%02x, /* %d */\n", 4, ' ',ptr->stretch_rm, ptr->stretch_rm);
    printf("%*c.drv_group_a_num = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_a_num.value, ptr->drv_group_a_num.value);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_a_num.reversed, ptr->drv_group_a_num.reversed);
    printf("%*c.all_driving = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_a_num.all_driving, ptr->drv_group_a_num.all_driving);
    printf("%*c},\n", 4, ' ');
    printf("%*c.drv_group_b_num = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_b_num.value, ptr->drv_group_b_num.value);
    printf("%*c.dual_freq = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_b_num.dual_freq, ptr->drv_group_b_num.dual_freq);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_group_b_num.reversed, ptr->drv_group_b_num.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.sensor_num = {\n", 4, ' ');
    printf("%*c.group_a_number = 0x%02x, /* %d */\n", 8, ' ',ptr->sensor_num.group_a_number, ptr->sensor_num.group_a_number);
    printf("%*c.group_b_number = 0x%02x, /* %d */\n", 8, ' ',ptr->sensor_num.group_b_number, ptr->sensor_num.group_b_number);
    printf("%*c},\n", 4, ' ');
    printf("%*c.freq_a_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->freq_a_factor, ptr->freq_a_factor);
    printf("%*c.freq_b_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->freq_b_factor, ptr->freq_b_factor);
    printf("%*c.pannel_bit_freq = 0x%04x, /* %d */\n", 4, ' ',ptr->pannel_bit_freq, ptr->pannel_bit_freq);
    printf("%*c.pannel_sensor_time = 0x%04x, /* %d */\n", 4, ' ',ptr->pannel_sensor_time, ptr->pannel_sensor_time);
    printf("%*c.pannel_tx_gain = {\n", 4, ' ');
    printf("%*c.dac_gain = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_tx_gain.dac_gain, ptr->pannel_tx_gain.dac_gain);
    printf("%*c.drv_output_r = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_tx_gain.drv_output_r, ptr->pannel_tx_gain.drv_output_r);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_tx_gain.reversed, ptr->pannel_tx_gain.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.pannel_rx_gain = {\n", 4, ' ');
    printf("%*c.pga_gain = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_rx_gain.pga_gain, ptr->pannel_rx_gain.pga_gain);
    printf("%*c.rx_vcmi = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_rx_gain.rx_vcmi, ptr->pannel_rx_gain.rx_vcmi);
    printf("%*c.pga_r = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_rx_gain.pga_r, ptr->pannel_rx_gain.pga_r);
    printf("%*c.pga_c = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_rx_gain.pga_c, ptr->pannel_rx_gain.pga_c);
    printf("%*c},\n", 4, ' ');
    printf("%*c.pannel_dump_shift = {\n", 4, ' ');
    printf("%*c.raw_touch = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_dump_shift.raw_touch, ptr->pannel_dump_shift.raw_touch);
    printf("%*c.raw_gesture = 0x%02x, /* %d */\n", 8, ' ',ptr->pannel_dump_shift.raw_gesture, ptr->pannel_dump_shift.raw_gesture);
    printf("%*c},\n", 4, ' ');
    printf("%*c.drv_frame_control = {\n", 4, ' ');
    printf("%*c.repeat_num = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_frame_control.repeat_num, ptr->drv_frame_control.repeat_num);
    printf("%*c.subframe_drv_num = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_frame_control.subframe_drv_num, ptr->drv_frame_control.subframe_drv_num);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->drv_frame_control.reversed, ptr->drv_frame_control.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.charging_level_up = 0x%02x, /* %d */\n", 4, ' ',ptr->charging_level_up, ptr->charging_level_up);
    printf("%*c.module_switch3 = {\n", 4, ' ');
    printf("%*c.shape_en = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch3.shape_en, ptr->module_switch3.shape_en);
    printf("%*c.reserved1 = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch3.reserved1, ptr->module_switch3.reserved1);
    printf("%*c.strong_smooth = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch3.strong_smooth, ptr->module_switch3.strong_smooth);
    printf("%*c.gesture_hop_dis = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch3.gesture_hop_dis, ptr->module_switch3.gesture_hop_dis);
    printf("%*c.reserved2 = 0x%02x, /* %d */\n", 8, ' ',ptr->module_switch3.reserved2, ptr->module_switch3.reserved2);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_dis = {\n", 4, ' ');
    printf("%*c.distance_left_right = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_dis.distance_left_right, ptr->gesture_dis.distance_left_right);
    printf("%*c.distance_up_down = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_dis.distance_up_down, ptr->gesture_dis.distance_up_down);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_long_press_time = 0x%02x, /* %d */\n", 4, ' ',ptr->gesture_long_press_time, ptr->gesture_long_press_time);
    printf("%*c.x_y_slop_adjust = {\n", 4, ' ');
    printf("%*c.y = 0x%02x, /* %d */\n", 8, ' ',ptr->x_y_slop_adjust.y, ptr->x_y_slop_adjust.y);
    printf("%*c.x = 0x%02x, /* %d */\n", 8, ' ',ptr->x_y_slop_adjust.x, ptr->x_y_slop_adjust.x);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_control = {\n", 4, ' ');
    printf("%*c.drv_pga_gain = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_control.drv_pga_gain, ptr->gesture_control.drv_pga_gain);
    printf("%*c.invalid_time = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_control.invalid_time, ptr->gesture_control.invalid_time);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_switch1 = {\n", 4, ' ');
    printf("%*c.c = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.c, ptr->gesture_switch1.c);
    printf("%*c.e = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.e, ptr->gesture_switch1.e);
    printf("%*c.m = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.m, ptr->gesture_switch1.m);
    printf("%*c.o = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.o, ptr->gesture_switch1.o);
    printf("%*c.w = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.w, ptr->gesture_switch1.w);
    printf("%*c.swipe_right = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.swipe_right, ptr->gesture_switch1.swipe_right);
    printf("%*c.swipe_up = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.swipe_up, ptr->gesture_switch1.swipe_up);
    printf("%*c.swipe_left = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch1.swipe_left, ptr->gesture_switch1.swipe_left);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_switch2 = {\n", 4, ' ');
    printf("%*c.swipe_down = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.swipe_down, ptr->gesture_switch2.swipe_down);
    printf("%*c.double_tap = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.double_tap, ptr->gesture_switch2.double_tap);
    printf("%*c.v = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.v, ptr->gesture_switch2.v);
    printf("%*c.gt = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.gt, ptr->gesture_switch2.gt);
    printf("%*c.arrow = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.arrow, ptr->gesture_switch2.arrow);
    printf("%*c.s = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.s, ptr->gesture_switch2.s);
    printf("%*c.z = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.z, ptr->gesture_switch2.z);
    printf("%*c.bottom_valid = 0x%02x, /* %d */\n", 8, ' ',ptr->gesture_switch2.bottom_valid, ptr->gesture_switch2.bottom_valid);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_refresh_rate = 0x%02x, /* %d */\n", 4, ' ',ptr->gesture_refresh_rate, ptr->gesture_refresh_rate);
    printf("%*c.gesture_touch_level = 0x%02x, /* %d */\n", 4, ' ',ptr->gesture_touch_level, ptr->gesture_touch_level);
    printf("%*c.new_green_wake_up_level = 0x%02x, /* %d */\n", 4, ' ',ptr->new_green_wake_up_level, ptr->new_green_wake_up_level);
    printf("%*c.freq_hopping_start = 0x%02x, /* %d */\n", 4, ' ',ptr->freq_hopping_start, ptr->freq_hopping_start);
    printf("%*c.freq_hopping_end = 0x%02x, /* %d */\n", 4, ' ',ptr->freq_hopping_end, ptr->freq_hopping_end);
    printf("%*c.noise_detect_times = {\n", 4, ' ');
    printf("%*c.confirm_times = 0x%02x, /* %d */\n", 8, ' ',ptr->noise_detect_times.confirm_times, ptr->noise_detect_times.confirm_times);
    printf("%*c.stay_times = 0x%02x, /* %d */\n", 8, ' ',ptr->noise_detect_times.stay_times, ptr->noise_detect_times.stay_times);
    printf("%*c},\n", 4, ' ');
    printf("%*c.hopping_flag = {\n", 4, ' ');
    printf("%*c.detect_time_out = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_flag.detect_time_out, ptr->hopping_flag.detect_time_out);
    printf("%*c.delay_hopping = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_flag.delay_hopping, ptr->hopping_flag.delay_hopping);
    printf("%*c.dis_force_ref = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_flag.dis_force_ref, ptr->hopping_flag.dis_force_ref);
    printf("%*c.range_ext = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_flag.range_ext, ptr->hopping_flag.range_ext);
    printf("%*c.hopping_en = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_flag.hopping_en, ptr->hopping_flag.hopping_en);
    printf("%*c},\n", 4, ' ');
    printf("%*c.hopping_threshold = {\n", 4, ' ');
    printf("%*c.hopping_hit_threshold = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_threshold.hopping_hit_threshold, ptr->hopping_threshold.hopping_hit_threshold);
    printf("%*c.large_noise_threshold = 0x%02x, /* %d */\n", 8, ' ',ptr->hopping_threshold.large_noise_threshold, ptr->hopping_threshold.large_noise_threshold);
    printf("%*c},\n", 4, ' ');
    printf("%*c.noise_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->noise_threshold, ptr->noise_threshold);
    printf("%*c.noise_min_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->noise_min_threshold, ptr->noise_min_threshold);
    printf("%*c.reserved_0x8081 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x8081, ptr->reserved_0x8081);
    printf("%*c.hopping_sensor_group = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_sensor_group, ptr->hopping_sensor_group);
    printf("%*c.hopping_seg1_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg1_normalize, ptr->hopping_seg1_normalize);
    printf("%*c.hopping_seg1_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg1_factor, ptr->hopping_seg1_factor);
    printf("%*c.main_clock_adjust = 0x%02x, /* %d */\n", 4, ' ',ptr->main_clock_adjust, ptr->main_clock_adjust);
    printf("%*c.hopping_seg2_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg2_normalize, ptr->hopping_seg2_normalize);
    printf("%*c.hopping_seg2_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg2_factor, ptr->hopping_seg2_factor);
    printf("%*c.reserved_0x8088 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x8088, ptr->reserved_0x8088);
    printf("%*c.hopping_seg3_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg3_normalize, ptr->hopping_seg3_normalize);
    printf("%*c.hopping_seg3_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg3_factor, ptr->hopping_seg3_factor);
    printf("%*c.reserved_0x808b = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x808b, ptr->reserved_0x808b);
    printf("%*c.hopping_seg4_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg4_normalize, ptr->hopping_seg4_normalize);
    printf("%*c.hopping_seg4_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg4_factor, ptr->hopping_seg4_factor);
    printf("%*c.reserved_0x808e = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x808e, ptr->reserved_0x808e);
    printf("%*c.hopping_seg5_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg5_normalize, ptr->hopping_seg5_normalize);
    printf("%*c.hopping_seg5_factor = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg5_factor, ptr->hopping_seg5_factor);
    printf("%*c.reserved_0x8091 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x8091, ptr->reserved_0x8091);
    printf("%*c.hopping_seg6_normalize = 0x%02x, /* %d */\n", 4, ' ',ptr->hopping_seg6_normalize, ptr->hopping_seg6_normalize);
    printf("%*c.key1 = 0x%02x, /* %d */\n", 4, ' ',ptr->key1, ptr->key1);
    printf("%*c.key2 = 0x%02x, /* %d */\n", 4, ' ',ptr->key2, ptr->key2);
    printf("%*c.key3 = 0x%02x, /* %d */\n", 4, ' ',ptr->key3, ptr->key3);
    printf("%*c.key4 = 0x%02x, /* %d */\n", 4, ' ',ptr->key4, ptr->key4);
    printf("%*c.key_area = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->key_area.value, ptr->key_area.value);
    printf("%*c.hold_update_time = 0x%02x, /* %d */\n", 8, ' ',ptr->key_area.hold_update_time, ptr->key_area.hold_update_time);
    printf("%*c},\n", 4, ' ');
    printf("%*c.key_touch_level = 0x%02x, /* %d */\n", 4, ' ',ptr->key_touch_level, ptr->key_touch_level);
    printf("%*c.key_leave_level = 0x%02x, /* %d */\n", 4, ' ',ptr->key_leave_level, ptr->key_leave_level);
    printf("%*c.key_sens = {\n", 4, ' ');
    printf("%*c.keysens_2 = 0x%02x, /* %d */\n", 8, ' ',ptr->key_sens.keysens_2, ptr->key_sens.keysens_2);
    printf("%*c.keysens_1 = 0x%02x, /* %d */\n", 8, ' ',ptr->key_sens.keysens_1, ptr->key_sens.keysens_1);
    printf("%*c.keysens_4 = 0x%02x, /* %d */\n", 8, ' ',ptr->key_sens.keysens_4, ptr->key_sens.keysens_4);
    printf("%*c.keysens_3 = 0x%02x, /* %d */\n", 8, ' ',ptr->key_sens.keysens_3, ptr->key_sens.keysens_3);
    printf("%*c},\n", 4, ' ');
    printf("%*c.key_restrain = {\n", 4, ' ');
    printf("%*c.independent = 0x%02x, /* %d */\n", 8, ' ',ptr->key_restrain.independent, ptr->key_restrain.independent);
    printf("%*c.leave_time = 0x%02x, /* %d */\n", 8, ' ',ptr->key_restrain.leave_time, ptr->key_restrain.leave_time);
    printf("%*c},\n", 4, ' ');
    printf("%*c.key_restrain_time = {\n", 4, ' ');
    printf("%*c.value = 0x%02x, /* %d */\n", 8, ' ',ptr->key_restrain_time.value, ptr->key_restrain_time.value);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->key_restrain_time.reversed, ptr->key_restrain_time.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.gesture_large_touch = 0x%02x, /* %d */\n", 4, ' ',ptr->gesture_large_touch, ptr->gesture_large_touch);
    printf("%*c.reserved_0x809f = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x809f, ptr->reserved_0x809f);
    printf("%*c.reserved_0x80a0 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x80a0, ptr->reserved_0x80a0);
    printf("%*c.hotknot_noise_map = {\n", 4, ' ');
    printf("%*c._450K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._450K, ptr->hotknot_noise_map._450K);
    printf("%*c._400K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._400K, ptr->hotknot_noise_map._400K);
    printf("%*c._350K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._350K, ptr->hotknot_noise_map._350K);
    printf("%*c._300K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._300K, ptr->hotknot_noise_map._300K);
    printf("%*c._250K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._250K, ptr->hotknot_noise_map._250K);
    printf("%*c._200K = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map._200K, ptr->hotknot_noise_map._200K);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->hotknot_noise_map.reversed, ptr->hotknot_noise_map.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.link_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->link_threshold, ptr->link_threshold);
    printf("%*c.pxy_threshold = 0x%02x, /* %d */\n", 4, ' ',ptr->pxy_threshold, ptr->pxy_threshold);
    printf("%*c.ghot_dump_shift = {\n", 4, ' ');
    printf("%*c.raw_data = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_dump_shift.raw_data, ptr->ghot_dump_shift.raw_data);
    printf("%*c.rx_self = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_dump_shift.rx_self, ptr->ghot_dump_shift.rx_self);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_dump_shift.reversed, ptr->ghot_dump_shift.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.ghot_rx_gain = {\n", 4, ' ');
    printf("%*c.pga_gain = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_rx_gain.pga_gain, ptr->ghot_rx_gain.pga_gain);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_rx_gain.reversed, ptr->ghot_rx_gain.reversed);
    printf("%*c.pga_r = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_rx_gain.pga_r, ptr->ghot_rx_gain.pga_r);
    printf("%*c.pga_c = 0x%02x, /* %d */\n", 8, ' ',ptr->ghot_rx_gain.pga_c, ptr->ghot_rx_gain.pga_c);
    printf("%*c},\n", 4, ' ');
    printf("%*c.freq_gain0 = {\n", 4, ' ');
    printf("%*c.gain_cal1 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain0.gain_cal1, ptr->freq_gain0.gain_cal1);
    printf("%*c.gain_cal2 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain0.gain_cal2, ptr->freq_gain0.gain_cal2);
    printf("%*c},\n", 4, ' ');
    printf("%*c.freq_gain1 = {\n", 4, ' ');
    printf("%*c.gain_cal1 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain1.gain_cal1, ptr->freq_gain1.gain_cal1);
    printf("%*c.gain_cal2 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain1.gain_cal2, ptr->freq_gain1.gain_cal2);
    printf("%*c},\n", 4, ' ');
    printf("%*c.freq_gain2 = {\n", 4, ' ');
    printf("%*c.gain_cal1 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain2.gain_cal1, ptr->freq_gain2.gain_cal1);
    printf("%*c.gain_cal2 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain2.gain_cal2, ptr->freq_gain2.gain_cal2);
    printf("%*c},\n", 4, ' ');
    printf("%*c.freq_gain3 = {\n", 4, ' ');
    printf("%*c.gain_cal1 = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain3.gain_cal1, ptr->freq_gain3.gain_cal1);
    printf("%*c.reversed = 0x%02x, /* %d */\n", 8, ' ',ptr->freq_gain3.reversed, ptr->freq_gain3.reversed);
    printf("%*c},\n", 4, ' ');
    printf("%*c.reserved_0x80aa_0x80b2 = {\n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */\n    },\n", 4, ' ',
        ptr->reserved_0x80aa_0x80b2[0], ptr->reserved_0x80aa_0x80b2[0], ptr->reserved_0x80aa_0x80b2[1], ptr->reserved_0x80aa_0x80b2[1], ptr->reserved_0x80aa_0x80b2[2], ptr->reserved_0x80aa_0x80b2[2], ptr->reserved_0x80aa_0x80b2[3], ptr->reserved_0x80aa_0x80b2[3], 
        ptr->reserved_0x80aa_0x80b2[4], ptr->reserved_0x80aa_0x80b2[4], ptr->reserved_0x80aa_0x80b2[5], ptr->reserved_0x80aa_0x80b2[5], ptr->reserved_0x80aa_0x80b2[6], ptr->reserved_0x80aa_0x80b2[6], ptr->reserved_0x80aa_0x80b2[7], ptr->reserved_0x80aa_0x80b2[7], 
        ptr->reserved_0x80aa_0x80b2[8], ptr->reserved_0x80aa_0x80b2[8]);
    printf("%*c.combine_dis = {\n", 4, ' ');
    printf("%*c.adj = 0x%02x, /* %d */\n", 8, ' ',ptr->combine_dis.adj, ptr->combine_dis.adj);
    printf("%*c.gesture_adj = 0x%02x, /* %d */\n", 8, ' ',ptr->combine_dis.gesture_adj, ptr->combine_dis.gesture_adj);
    printf("%*c},\n", 4, ' ');
    printf("%*c.split_set = {\n", 4, ' ');
    printf("%*c.normal_size = 0x%02x, /* %d */\n", 8, ' ',ptr->split_set.normal_size, ptr->split_set.normal_size);
    printf("%*c.large_area = 0x%02x, /* %d */\n", 8, ' ',ptr->split_set.large_area, ptr->split_set.large_area);
    printf("%*c},\n", 4, ' ');
    printf("%*c.reserved_0x80b5 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x80b5, ptr->reserved_0x80b5);
    printf("%*c.reserved_0x80b6 = 0x%02x, /* %d */\n", 4, ' ',ptr->reserved_0x80b6, ptr->reserved_0x80b6);
    printf("%*c.sensor_ch = {\n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */\n    },\n", 4, ' ',
        ptr->sensor_ch[0], ptr->sensor_ch[0], ptr->sensor_ch[1], ptr->sensor_ch[1], ptr->sensor_ch[2], ptr->sensor_ch[2], ptr->sensor_ch[3], ptr->sensor_ch[3], 
        ptr->sensor_ch[4], ptr->sensor_ch[4], ptr->sensor_ch[5], ptr->sensor_ch[5], ptr->sensor_ch[6], ptr->sensor_ch[6], ptr->sensor_ch[7], ptr->sensor_ch[7], 
        ptr->sensor_ch[8], ptr->sensor_ch[8], ptr->sensor_ch[9], ptr->sensor_ch[9], ptr->sensor_ch[10], ptr->sensor_ch[10], ptr->sensor_ch[11], ptr->sensor_ch[11], 
        ptr->sensor_ch[12], ptr->sensor_ch[12], ptr->sensor_ch[13], ptr->sensor_ch[13]);
    printf("%*c.reserved_0x80c5_0x80d4 = {\n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */\n    },\n", 4, ' ',
        ptr->reserved_0x80c5_0x80d4[0], ptr->reserved_0x80c5_0x80d4[0], ptr->reserved_0x80c5_0x80d4[1], ptr->reserved_0x80c5_0x80d4[1], ptr->reserved_0x80c5_0x80d4[2], ptr->reserved_0x80c5_0x80d4[2], ptr->reserved_0x80c5_0x80d4[3], ptr->reserved_0x80c5_0x80d4[3], 
        ptr->reserved_0x80c5_0x80d4[4], ptr->reserved_0x80c5_0x80d4[4], ptr->reserved_0x80c5_0x80d4[5], ptr->reserved_0x80c5_0x80d4[5], ptr->reserved_0x80c5_0x80d4[6], ptr->reserved_0x80c5_0x80d4[6], ptr->reserved_0x80c5_0x80d4[7], ptr->reserved_0x80c5_0x80d4[7], 
        ptr->reserved_0x80c5_0x80d4[8], ptr->reserved_0x80c5_0x80d4[8], ptr->reserved_0x80c5_0x80d4[9], ptr->reserved_0x80c5_0x80d4[9], ptr->reserved_0x80c5_0x80d4[10], ptr->reserved_0x80c5_0x80d4[10], ptr->reserved_0x80c5_0x80d4[11], ptr->reserved_0x80c5_0x80d4[11], 
        ptr->reserved_0x80c5_0x80d4[12], ptr->reserved_0x80c5_0x80d4[12], ptr->reserved_0x80c5_0x80d4[13], ptr->reserved_0x80c5_0x80d4[13], ptr->reserved_0x80c5_0x80d4[14], ptr->reserved_0x80c5_0x80d4[14], ptr->reserved_0x80c5_0x80d4[15], ptr->reserved_0x80c5_0x80d4[15]);
    printf("%*c.driver_ch = {\n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */\n    },\n", 4, ' ',
        ptr->driver_ch[0], ptr->driver_ch[0], ptr->driver_ch[1], ptr->driver_ch[1], ptr->driver_ch[2], ptr->driver_ch[2], ptr->driver_ch[3], ptr->driver_ch[3], 
        ptr->driver_ch[4], ptr->driver_ch[4], ptr->driver_ch[5], ptr->driver_ch[5], ptr->driver_ch[6], ptr->driver_ch[6], ptr->driver_ch[7], ptr->driver_ch[7], 
        ptr->driver_ch[8], ptr->driver_ch[8], ptr->driver_ch[9], ptr->driver_ch[9], ptr->driver_ch[10], ptr->driver_ch[10], ptr->driver_ch[11], ptr->driver_ch[11], 
        ptr->driver_ch[12], ptr->driver_ch[12], ptr->driver_ch[13], ptr->driver_ch[13], ptr->driver_ch[14], ptr->driver_ch[14], ptr->driver_ch[15], ptr->driver_ch[15], 
        ptr->driver_ch[16], ptr->driver_ch[16], ptr->driver_ch[17], ptr->driver_ch[17], ptr->driver_ch[18], ptr->driver_ch[18], ptr->driver_ch[19], ptr->driver_ch[19], 
        ptr->driver_ch[20], ptr->driver_ch[20], ptr->driver_ch[21], ptr->driver_ch[21], ptr->driver_ch[22], ptr->driver_ch[22], ptr->driver_ch[23], ptr->driver_ch[23], 
        ptr->driver_ch[24], ptr->driver_ch[24], ptr->driver_ch[25], ptr->driver_ch[25]);
    printf("%*c.reserved_0x80ef_0x80fe = {\n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, \n        0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */, 0x%02x /* %d */\n    },\n", 4, ' ',
        ptr->reserved_0x80ef_0x80fe[0], ptr->reserved_0x80ef_0x80fe[0], ptr->reserved_0x80ef_0x80fe[1], ptr->reserved_0x80ef_0x80fe[1], ptr->reserved_0x80ef_0x80fe[2], ptr->reserved_0x80ef_0x80fe[2], ptr->reserved_0x80ef_0x80fe[3], ptr->reserved_0x80ef_0x80fe[3], 
        ptr->reserved_0x80ef_0x80fe[4], ptr->reserved_0x80ef_0x80fe[4], ptr->reserved_0x80ef_0x80fe[5], ptr->reserved_0x80ef_0x80fe[5], ptr->reserved_0x80ef_0x80fe[6], ptr->reserved_0x80ef_0x80fe[6], ptr->reserved_0x80ef_0x80fe[7], ptr->reserved_0x80ef_0x80fe[7], 
        ptr->reserved_0x80ef_0x80fe[8], ptr->reserved_0x80ef_0x80fe[8], ptr->reserved_0x80ef_0x80fe[9], ptr->reserved_0x80ef_0x80fe[9], ptr->reserved_0x80ef_0x80fe[10], ptr->reserved_0x80ef_0x80fe[10], ptr->reserved_0x80ef_0x80fe[11], ptr->reserved_0x80ef_0x80fe[11], 
        ptr->reserved_0x80ef_0x80fe[12], ptr->reserved_0x80ef_0x80fe[12], ptr->reserved_0x80ef_0x80fe[13], ptr->reserved_0x80ef_0x80fe[13], ptr->reserved_0x80ef_0x80fe[14], ptr->reserved_0x80ef_0x80fe[14], ptr->reserved_0x80ef_0x80fe[15], ptr->reserved_0x80ef_0x80fe[15]);
    printf("%*c.config_checksum = 0x%02x, /* %d */\n", 4, ' ',ptr->config_checksum, ptr->config_checksum);
    printf("%*c.config_fresh = 0x%02x, /* %d */\n", 4, ' ',ptr->config_fresh, ptr->config_fresh);
    printf("}\n");
}



static void hal_i2c_scan()
{
    uint8_t foundDevices = 0;
  	for(int address = 1; address < 127; address++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
      foundDevices++;
		}
		i2c_cmd_link_delete(cmd);
	  }
    if (foundDevices == 0)
    {
      printf("-> found NO devices");
    }
}


static void hal_hex_dump(uint8_t *data, size_t len){
    for(size_t i=1;i<=len;i++){
        printf("%02x ", data[i-1]);
        if(i !=0 && i%16 == 0)
            printf("\n");
    }
    printf("\nSize:%d \n", len);
}
