#ifndef GT911_HAL_H
#define GT911_HAL_H

#include <stdint.h>
#include "lvgl.h"


#define PRT_MEMBER(n, type, x, y) do{ \
    if(sizeof(type) == sizeof(uint8_t)) \
        printf("%*c" #type " "  #x  "->" #y ": 0x%02x\n", n*4, ' ', (x).y); \
    if(sizeof(type) == sizeof(uint16_t)) \
        printf("%*c" #type " "  #x  "->" #y ": 0x%04x\n", n*4, ' ', (x).y); \
    if(sizeof(type) == sizeof(uint32_t)) \
        printf("%*c" #type " "  #x  "->" #y ": 0x%08x\n", n*4, ' ', (x).y); \
}while(0)

#define PRT_STRUCT_START(x) do{ \
    const char *name = #x; \
    int i = 0;

#define PRT_STRUCT_END() }while(0)

#define GT911_REG(x)   (uint8_t)(x>>8), (uint8_t)(x & 0xff)

#define GT911_FIMWARE_VERSION       (0x1040U)

#define GT911_REG_COMMAND           (0x8040U)
/*
* ESD protection mechanism enabled;
* reset to 0 upon initialization;
* after that, driver writes 0xAA to 0x8040 and reads and checks the value of 0x8040 regularly.
*/
#define GT911_REG_ESD_CHECK         (0x8041U)
/*
* For commands greater than 0x07, 
* it is required to write the command to 0x8046 before writing to 0x8040, to improve anti-ESD capability.
*/
#define GT911_REG_COMMAND_CHECK     (0x8041U)

#define GT911_REG_PRODUCT_INFO      (0x8140U)
#define GT911_REG_POINT_INFO        (0x814EU)
#define GT911_REG_POINT_BASE        (0x814FU)
#define GT911_REG_CONFIG_BASE       (0x8047U)

typedef enum {
    GT911_ADDR_BABB = 0,
    GT911_ADDR_2829
} gt911_addr_t;

typedef enum {
    /* 0x8040 */
    GT911_READ_COORDINATE = 0,
    GT911_READ_DIFF_DATA,
    GT911_SOFT_RESET,
    GT911_INTERNAL_UPDATE_REF_CAP,
    GT911_INTERNAL_CALIBRATION_REF_CAP,
    GT911_SCREEN_OFF,
    GT911_ENTER_CHARGE_MODE,
    GT911_EXIT_CHARGE_MODE,
    GT911_GESTURE_MODE,
    GT911_ENTER_HOTKNOT_SLAVE_APPROACH_MODE = 0x20,
    GT911_ENTER_HOTKNOT_MASTER_APPROACH_MODE,
    GT911_ENTER_RECEOVE_MODE,
    GT911_EXIT_HOTKNOT_SLAVE_APPROACH_MODE = 0x28,
    GT911_EXIT_HOTKNOT_MASTER_APPROACH_MODE,
    GT911_EXIT_RECEOVE_MODE,
    GT911_ENABLE_ESD_PROTECTION = 0xAA,
} gt911_command_t;

/* size: 56 */
typedef struct __attribute__((packed)) {
    /* 0x8140 */
    char product_id[4];
    uint16_t firmware_version;
    uint16_t x_coordinate_resolution;
    uint16_t y_coordinate_resolution;
    uint8_t vendor_id;
    uint8_t reserved_0x814b;
    uint8_t reserved_0x814c;
    uint8_t reserved_0x814d;
    /* 0x814E */
    struct __attribute__((packed)) {
        uint8_t total_points: 4;
        uint8_t reserved: 2;
        uint8_t large_detect: 1;
        uint8_t buffer_status: 1;
    } info;
    struct __attribute__((packed)) {
        /* 0x814F + (index * 8) */
        uint8_t track_id;
        uint16_t x;
        uint16_t y;
        uint16_t size;
        uint8_t reserved;
    } points[5];
    uint8_t reserved_0x8177;
} gt911_coordinate_info_t;

/* size: 186 */
typedef struct __attribute__((packed)) {
    /* 0x8047 */
    uint8_t config_version;     /* From 'A' to 'Z'. Set 0x00 and the version number is reset to 'A' */
    uint16_t x_output_max;      /* Resolution of X axis */
    uint16_t y_output_max;      /* Resolution of Y axis */
    struct __attribute__((packed)) {
        uint8_t value: 4;       /* Touch points supported: 1 to 5 */
        uint8_t reserved: 4;
    } touch_number;
    struct __attribute__((packed)) {
        uint8_t int_mode: 2;    /* INT triggering mechanism. 00: rising edge 01: falling edge 02: Low level 03: High level */
        uint8_t sito: 1;        /* Software noise reduction */
        uint8_t x2y_mode: 1;
        uint8_t stretch_rank: 2;
        uint8_t sensor_reseral: 1;  /* X2X */
        uint8_t driver_reseral: 1;  /* Y2Y */
    } module_switch1;
    struct __attribute__((packed)) {
        uint8_t touch_key: 1;
        uint8_t hotknot_en: 1;
        uint8_t approch_en: 1;
        uint8_t reserved1: 2;
        uint8_t first_filte_dis: 1;
        uint8_t reserved2: 2;
    } module_switch2;
    struct __attribute__((packed)) {
        uint8_t pressing: 4;    /* De-jitter frequency when touch is pressing down */
        uint8_t released: 4;    /* De-jitter frequency when touch is being released */
    } shake_count;
    struct __attribute__((packed)) {
        uint8_t normal_filter: 6;   /* Filter threshold for original coordinates, coefficient is 4 */
        uint8_t first_filter: 2;
    } filter;
    uint8_t large_touch;    /* Number of large-area touch points */
    struct __attribute__((packed)) {
        uint8_t value: 4;       /* Noise reduction value (0-15 valid, coefficient is 1) */
        uint8_t reserved: 4;
    } noise_reduction;
    /* 0x8053 */
    uint8_t screen_touch_level;     /* Threshold for touch to be detected */
    uint8_t screen_leave_level;     /* Threshold for touch to be released */
    struct __attribute__((packed)) {
        uint8_t value: 4;       /* Interval to enter lower power consumption mode (0s to 15s) */
        uint8_t reserved: 4;
    } low_power_control;
    struct __attribute__((packed)) {
        uint8_t report_rate: 4;     /* Coordinates report rate (period: 5+N ms) */
        uint8_t pluse_width: 4;     /* Pulse width setting for gesture wakeup */
    } refresh_rate;
    uint8_t x_threshold;    /* X coordinate output threshold: 0-255 (Based on the last reported coordinates. If configured to 0, GT911 will keep outputting coordinates continuously) */
    uint8_t y_threshold;    /* Y coordinate output threshold: 0-255 (Based on the last reported coordinates. If configured to 0, GT911 will keep outputting coordinates continuously) */
    uint8_t x_speed_limit;  /* Reserved */
    uint8_t y_speed_limit;  /* Reserved */
    struct __attribute__((packed)) {
        uint8_t bottom: 4;  /* Space of border bottom (coefficient: 32) */
        uint8_t top: 4;     /* Space of border top (coefficient: 32) */
        uint8_t right: 4;   /* Space of border right (coefficient: 32) */
        uint8_t left: 4;    /* Space of border left (coefficient: 32) */
    } space;
    struct __attribute__((packed)) {
        uint8_t value: 4;       /* Mini filter configuration during line drawing process, configured as 0 indicates 4 */
        uint8_t reserved: 4;
    } mini_filter;
    uint8_t stretch_r0;     /* coefficient of Stretch space 1 */
    uint8_t stretch_r1;     /* coefficient of Stretch space 2 */
    uint8_t stretch_r2;     /* coefficient of Stretch space 3 */
    uint8_t stretch_rm;     /* The base of multiple stretch spaces */
    struct __attribute__((packed)) {
        uint8_t value: 5;       /* Driver_Group_A_number */
        uint8_t reversed: 2;
        uint8_t all_driving: 1;
    } drv_group_a_num;
    struct __attribute__((packed)) {
        uint8_t value: 5;       /* Driver_Group_B_number */
        uint8_t dual_freq: 1;
        uint8_t reversed: 2;
    } drv_group_b_num;
    struct __attribute__((packed)) {
        uint8_t group_a_number: 4;
        uint8_t group_b_number: 4;
    } sensor_num;
    uint8_t freq_a_factor;          /* Clock Multiplier Factor of drive frequency of Driver Group A GroupA_Frequence = Clock Multiplier Factor * Fundamental Frequency */
    uint8_t freq_b_factor;          /* Clock Multiplier Factor of drive frequency of Driver Group B GroupB_Frequence = Clock Multiplier Factor * Fundamental Frequency */
    uint16_t pannel_bit_freq;       /* Fundamental Frequency of Driver Groups A and B (1526HZ< Fundamental Frequency <14600Hz) */
    uint16_t pannel_sensor_time;    /* Output Interval between two adjacent drive signals (unit: us); Reserved ( used in beta version; invalid in a Release) */
    struct __attribute__((packed)) {
        uint8_t dac_gain: 3;        /* 0: Gain max. 7: Gain min. */
        uint8_t drv_output_r: 2;    /* 4 gain values, configurable */
        uint8_t reversed: 3;
    } pannel_tx_gain;
    struct __attribute__((packed)) {
        uint8_t pga_gain: 3;    /* 8 gain values, configurable */
        uint8_t rx_vcmi: 2;     /* 4 gain values, configurable */
        uint8_t pga_r: 2;
        uint8_t pga_c: 1;
    } pannel_rx_gain;
    struct __attribute__((packed)) {
        uint8_t raw_touch: 4;   /* Amplification factor of raw data on the touch panel (2N) */
        uint8_t raw_gesture: 4; /* Amplification factor of raw data in Gesture Mode (2N) */
    } pannel_dump_shift;
    struct __attribute__((packed)) {
        uint8_t repeat_num: 2;          /* Accumulated sampling count */
        uint8_t subframe_drv_num: 5;
        uint8_t reversed: 1;
    } drv_frame_control;
    uint8_t charging_level_up;
    struct __attribute__((packed)) {
        uint8_t shape_en: 1;
        uint8_t reserved1: 4;
        uint8_t strong_smooth: 1;
        uint8_t gesture_hop_dis: 1;
        uint8_t reserved2: 1;
    } module_switch3;
    struct __attribute__((packed)) {
        uint8_t distance_left_right: 4;     /* Valid distance for slide-left/right wakeup */
        uint8_t distance_up_down: 4;        /* Valid distance for slide-up/down wakeup  */
    } gesture_dis;
    uint8_t gesture_long_press_time;    /* The gesture recognizing processing aborting time period when long touching */
    struct __attribute__((packed)) {
        uint8_t y: 4;   /* The adjustment parameter of Y direction slope when using “four point trigonometric approximation algorithm” to calculate the coordinates (0: algorithm disabled) */
        uint8_t x: 4;   /* The adjustment parameter of X direction slope when using “four point trigonometric approximation algorithm” to calculate the coordinates (0: algorithm disabled) */
    } x_y_slop_adjust;
    struct __attribute__((packed)) {
        uint8_t drv_pga_gain: 4; /* 8 gain values, configurable */
        uint8_t invalid_time: 4; /* Invalid time for double-tap wakeup (unit:100ms, defaults to 1.5s when configured as 0) */
    } gesture_control;
    struct __attribute__((packed)) {
        uint8_t c: 1;
        uint8_t e: 1;
        uint8_t m: 1;
        uint8_t o: 1;
        uint8_t w: 1;
        uint8_t swipe_right: 1;
        uint8_t swipe_up: 1;
        uint8_t swipe_left: 1;
    } gesture_switch1;
    struct __attribute__((packed)) {
        uint8_t swipe_down: 1;
        uint8_t double_tap: 1;
        uint8_t v: 1;
        uint8_t gt: 1;      /* > */
        uint8_t arrow: 1;   /* ^ */
        uint8_t s: 1;
        uint8_t z: 1;
        uint8_t bottom_valid: 1;    /* Swipe is valid only at the bottom of the TP */
    } gesture_switch2;
    uint8_t gesture_refresh_rate;       /* Report rate in Gesture mode (period is 5+ms) */
    uint8_t gesture_touch_level;        /* Touch threshold in Gesture mode */
    uint8_t new_green_wake_up_level;    /* Threshold for NewGreen wakeup of Gesture wakeup function */
    uint8_t freq_hopping_start;         /* Start frequency for frequency hopping( when Range_Ext=0，the unit is 2KHz，for example, 50 indicates 100KHz; When Range_Ext=1，the unit is BitFreq) */
    uint8_t freq_hopping_end;           /* End frequency for frequency hopping( when Range_Ext=0，the unit is 2KHz，for example, 150 indicates 300KHz; When Range_Ext=1，the unit is BitFreq) */
    struct __attribute__((packed)) {
        uint8_t confirm_times: 5;   /* (Confirmed noise level after repeated noise tests, 1-63 valid; 20 is recommended */
        uint8_t stay_times: 3;      /* (Number of tests taken on each frequency point in each noise test; 2 is recommended */
    } noise_detect_times;
    struct __attribute__((packed)) {
        uint8_t detect_time_out: 2; 
        uint8_t delay_hopping: 2; 
        uint8_t dis_force_ref: 1; 
        uint8_t range_ext: 2;
        uint8_t hopping_en: 1;
    } hopping_flag;
    struct __attribute__((packed)) {
        uint8_t hopping_hit_threshold: 5;
        uint8_t large_noise_threshold: 3;
    } hopping_threshold;
    uint8_t noise_threshold;
    uint8_t noise_min_threshold;
    uint8_t reserved_0x8081;
    uint8_t hopping_sensor_group;       /* Sections for Hopping Frequency Noise Detection (4 sections recommended) */
    uint8_t hopping_seg1_normalize;     /* Seg1 Normalize coefficient ( sampling value *N / 128= Raw data) */
    uint8_t hopping_seg1_factor;        /* Seg1 Central point Factor */
    uint8_t main_clock_adjust;          /* Fine adjustment of IC main clock Frequency, within the range of -7 to +8 */
    uint8_t hopping_seg2_normalize;     /* Seg2 Normalize coefficient (sampling value *N / 128= Raw data) */
    uint8_t hopping_seg2_factor;        /* Seg2 Central point Factor */
    uint8_t reserved_0x8088;
    uint8_t hopping_seg3_normalize;     /* Seg3 Normalize coefficient (sampling value *N / 128= Raw data) */
    uint8_t hopping_seg3_factor;        /* Seg3 Central point Factor */
    uint8_t reserved_0x808b;
    uint8_t hopping_seg4_normalize;     /* Seg4 Normalize coefficient (sampling value *N / 128= Raw data) */
    uint8_t hopping_seg4_factor;        /* Seg4 Central point Factor */
    uint8_t reserved_0x808e;
    uint8_t hopping_seg5_normalize;     /* Seg5 Normalize coefficient (sampling value *N / 128= Raw data) */
    uint8_t hopping_seg5_factor;        /* Seg5 Central point Factor */
    uint8_t reserved_0x8091;
    uint8_t hopping_seg6_normalize;     /* Seg6 Normalize coefficient (sampling value *N / 128= Raw data) */
    uint8_t key1;   /* Key 1 address: 0-255 valid (0 indicates no key is available. When the addresses of all four keys are the multiples of 8, it means independent key design manner. ) */
    uint8_t key2;   /* Key 2 address: 0-255 valid (0 indicates no key is available. When the addresses of all four keys are the multiples of 8, it means independent key design manner. ) */
    uint8_t key3;   /* Key 3 address: 0-255 valid (0 indicates no key is available. When the addresses of all four keys are the multiples of 8, it means independent key design manner. ) */
    uint8_t key4;   /* Key 4 address: 0-255 valid (0 indicates no key is available. When the addresses of all four keys are the multiples of 8, it means independent key design manner. ) */
    struct __attribute__((packed)) {
        uint8_t value: 4;               /* Key active area configuration (single side): 0-15 valid */
        uint8_t hold_update_time: 4;    /* Time limit for long-press update (1s to 15s). Long-press update is disabled when configured to 0. */
    } key_area;
    /* 0x8098 */
    uint8_t key_touch_level;    /* Touch key touch threshold */
    uint8_t key_leave_level;    /* Touch key release threshold */
    struct __attribute__((packed)) {
        uint8_t keysens_2: 4;   /* KeySens_2 (sensitivity coefficient of Key 2) */
        uint8_t keysens_1: 4;   /* KeySens_1 (sensitivity coefficient of Key 1) */
        uint8_t keysens_4: 4;   /* KeySens_4 (sensitivity coefficient of Key 4) */
        uint8_t keysens_3: 4;   /* KeySens_3 (sensitivity coefficient of Key 3) */
    } key_sens;
    struct __attribute__((packed)) {
        uint8_t independent: 4;     /* Independent adjacent key restrain parameter */
        uint8_t leave_time: 4;      /* The key restrain interval after finger leaves screen (unit: 100ms), 0 means the key suppression interval is 600ms. */
    } key_restrain;
    struct __attribute__((packed)) {
        uint8_t value: 4;
        uint8_t reversed: 4;
    } key_restrain_time;
    uint8_t gesture_large_touch;    /* Large-area touch processing in Gesture mode (the size of the touch rectangle). Configured as 0, this function is disabled. */
    uint8_t reserved_0x809f;
    uint8_t reserved_0x80a0;
    struct __attribute__((packed)) {
        uint8_t _450K: 1;
        uint8_t _400K: 1;
        uint8_t _350K: 1;
        uint8_t _300K: 1;
        uint8_t _250K: 1;
        uint8_t _200K: 1;
        uint8_t reversed: 2;
    } hotknot_noise_map;
    uint8_t link_threshold;     /* Link_NoiseThreshold */
    uint8_t pxy_threshold;      /* Pxy_NoiseThreshold */
    struct __attribute__((packed)) {
        uint8_t raw_data: 4;    /* Amplification factor of raw Data (2N) */
        uint8_t rx_self: 1;
        uint8_t reversed: 3;
    } ghot_dump_shift;
    struct __attribute__((packed)) {
        uint8_t pga_gain: 3;    /* PGA_Gain (8 levels to be configured) */
        uint8_t reversed: 2;
        uint8_t pga_r: 2;
        uint8_t pga_c: 1;
    } ghot_rx_gain;
    struct __attribute__((packed)) {
        uint8_t gain_cal1: 4;   /* 450K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
        uint8_t gain_cal2: 4;   /* 450K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
    } freq_gain0;
    struct __attribute__((packed)) {
        uint8_t gain_cal1: 4;   /* 350K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
        uint8_t gain_cal2: 4;   /* 350K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
    } freq_gain1;
    struct __attribute__((packed)) {
        uint8_t gain_cal1: 4;   /* 250K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
        uint8_t gain_cal2: 4;   /* 250K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
    } freq_gain2;
    struct __attribute__((packed)) {
        uint8_t gain_cal1: 4;   /* 150K signal gain calibration, calibration volume is N/16. Invalid when N=0. */
        uint8_t reversed: 4;
    } freq_gain3;
    uint8_t reserved_0x80aa_0x80b2[9];
    /* 0x80B3 */
    struct __attribute__((packed)) {
        uint8_t adj: 4;            /* Distance for adjacent rectangles to be combined */
        uint8_t gesture_adj: 4;    /* Distance for adjacent rectangles to be combined in Gesture mode */
    } combine_dis;
    struct __attribute__((packed)) {
        uint8_t normal_size: 4;     /* Distance for a normal-size rectangle to be split */
        uint8_t large_area: 4;      /* Distance for a large-area rectangle to be split */
    } split_set;
    uint8_t reserved_0x80b5;
    uint8_t reserved_0x80b6;
    uint8_t sensor_ch[14];      /* Channel number on chip corresponding to ITO Sensor */
    uint8_t reserved_0x80c5_0x80d4[16];
    uint8_t driver_ch[26];      /* Channel number on chip corresponding to ITO Driver */
    uint8_t reserved_0x80ef_0x80fe[16];
    uint8_t config_checksum;    /* Configuration verification (checksum value of the bytes from 0x8047 to 0x80FE) */
    uint8_t config_fresh;       /* Configuration updated flag (the flag is written by the host) */
} gt911_config_t;

typedef struct {
    uint8_t  i2c_port;
    uint16_t i2c_sda_pin;
    uint16_t i2c_scl_pin;
    uint32_t i2c_speed;
    uint16_t rst_pin;
    uint16_t int_pin;
    // uint8_t (*gt911_int_callback)(gt911_instance_t *instance);
} gt911_init_config_t;

typedef struct {
    gt911_init_config_t *config;

    uint8_t is_initialized;
    uint8_t i2c_addr;

    gt911_config_t cfg;
    gt911_coordinate_info_t info;
} gt911_instance_t;



int gt911_init(gt911_init_config_t *config);

inline uint8_t gt911_isready();
inline int gt911_read_config(gt911_config_t *config);
inline int gt911_write_config(gt911_config_t *config);
int gt911_overwrite_config(gt911_config_t *config);
uint8_t gt911_calc_checksum(gt911_config_t *config);

inline int gt911_read_info(gt911_coordinate_info_t *info);
inline int gt911_read_points(gt911_coordinate_info_t *info);

inline int gt911_send_command(gt911_command_t command);


bool gt911_lv_indev_cb(lv_indev_drv_t *drv, lv_indev_data_t *data);

void gt911_reset_config(gt911_config_t *config);
#endif //GT911_HAL_H
