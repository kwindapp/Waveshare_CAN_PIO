/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#define ESP_PANEL_USE_1024_600_LCD           (0)     // 0: 800x480, 1: 1024x600
#define ESP_OPEN_TOUCH 1// 1 initiates the touch, 0 closes the touch.

/**
 * @brief Flag to enable custom board configuration (0/1)
 */
#define ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM  (1)

#if ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM

#if ESP_PANEL_USE_1024_600_LCD
    #define ESP_PANEL_BOARD_NAME                "Waveshare:ESP32-S3-Touch-LCD-5B"
#else
    #define ESP_PANEL_BOARD_NAME                "Waveshare:ESP32-S3-Touch-LCD-5"
#endif

#if ESP_PANEL_USE_1024_600_LCD
    #define ESP_PANEL_BOARD_WIDTH         (1024)
    #define ESP_PANEL_BOARD_HEIGHT        (600)
#else
    #define ESP_PANEL_BOARD_WIDTH         (800)
    #define ESP_PANEL_BOARD_HEIGHT        (480)
#endif

#define ESP_PANEL_BOARD_USE_LCD             (1)

#if ESP_PANEL_BOARD_USE_LCD

#define ESP_PANEL_BOARD_LCD_CONTROLLER      ST7262
#define ESP_PANEL_BOARD_LCD_BUS_TYPE        (ESP_PANEL_BUS_TYPE_RGB)

#if ESP_PANEL_BOARD_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB

#define ESP_PANEL_BOARD_LCD_RGB_USE_CONTROL_PANEL       (0)

#if ESP_PANEL_USE_1024_600_LCD
    #define ESP_PANEL_BOARD_LCD_RGB_CLK_HZ            (21 * 1000 * 1000)
    #define ESP_PANEL_BOARD_LCD_RGB_HPW               (30)
    #define ESP_PANEL_BOARD_LCD_RGB_HBP               (145)
    #define ESP_PANEL_BOARD_LCD_RGB_HFP               (170)
    #define ESP_PANEL_BOARD_LCD_RGB_VPW               (2)
    #define ESP_PANEL_BOARD_LCD_RGB_VBP               (23)
    #define ESP_PANEL_BOARD_LCD_RGB_VFP               (12)
#else
    #define ESP_PANEL_BOARD_LCD_RGB_CLK_HZ            (16 * 1000 * 1000)
    #define ESP_PANEL_BOARD_LCD_RGB_HPW               (4)
    #define ESP_PANEL_BOARD_LCD_RGB_HBP               (8)
    #define ESP_PANEL_BOARD_LCD_RGB_HFP               (8)
    #define ESP_PANEL_BOARD_LCD_RGB_VPW               (4)
    #define ESP_PANEL_BOARD_LCD_RGB_VBP               (16)
    #define ESP_PANEL_BOARD_LCD_RGB_VFP               (16)
#endif

#define ESP_PANEL_BOARD_LCD_RGB_PCLK_ACTIVE_NEG (1)
#define ESP_PANEL_BOARD_LCD_RGB_DATA_WIDTH      (16)
#define ESP_PANEL_BOARD_LCD_RGB_PIXEL_BITS      (ESP_PANEL_LCD_COLOR_BITS_RGB565)
#define ESP_PANEL_BOARD_LCD_RGB_BOUNCE_BUF_SIZE (ESP_PANEL_BOARD_WIDTH * 10)

#define ESP_PANEL_BOARD_LCD_RGB_IO_HSYNC        (46)
#define ESP_PANEL_BOARD_LCD_RGB_IO_VSYNC        (3)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DE           (5)
#define ESP_PANEL_BOARD_LCD_RGB_IO_PCLK         (7)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DISP         (-1)

#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA0        (14)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA1        (38)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA2        (18)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA3        (17)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA4        (10)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA5        (39)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA6        (0)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA7        (45)

#if ESP_PANEL_BOARD_LCD_RGB_DATA_WIDTH > 8
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA8        (48)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA9        (47)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA10       (21)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA11       (1)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA12       (2)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA13       (42)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA14       (41)
#define ESP_PANEL_BOARD_LCD_RGB_IO_DATA15       (40)
#endif

#endif // RGB BUS

#define ESP_PANEL_BOARD_LCD_COLOR_BITS          (ESP_PANEL_LCD_COLOR_BITS_RGB565)
#define ESP_PANEL_BOARD_LCD_COLOR_BGR_ORDER     (0)
#define ESP_PANEL_BOARD_LCD_COLOR_INEVRT_BIT    (0)

#define ESP_PANEL_BOARD_LCD_SWAP_XY             (0)
#define ESP_PANEL_BOARD_LCD_MIRROR_X            (0)
#define ESP_PANEL_BOARD_LCD_MIRROR_Y            (0)
#define ESP_PANEL_BOARD_LCD_GAP_X               (0)
#define ESP_PANEL_BOARD_LCD_GAP_Y               (0)

#define ESP_PANEL_BOARD_LCD_RST_IO              (-1)
#define ESP_PANEL_BOARD_LCD_RST_LEVEL           (0)

#endif // USE_LCD

// ---------------- TOUCH -------------------

#define ESP_PANEL_BOARD_USE_TOUCH               (ESP_OPEN_TOUCH)

#if ESP_PANEL_BOARD_USE_TOUCH

#define ESP_PANEL_BOARD_TOUCH_CONTROLLER        GT911
#define ESP_PANEL_BOARD_TOUCH_BUS_TYPE          (ESP_PANEL_BUS_TYPE_I2C)

#define ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST        (0)

#if ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_I2C

#define ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID           (0)
#define ESP_PANEL_BOARD_TOUCH_I2C_CLK_HZ            (400 * 1000)
#define ESP_PANEL_BOARD_TOUCH_I2C_SCL_PULLUP        (1)
#define ESP_PANEL_BOARD_TOUCH_I2C_SDA_PULLUP        (1)
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL            (9)
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA            (8)

#define ESP_PANEL_BOARD_TOUCH_I2C_ADDRESS           (0)

#endif

#define ESP_PANEL_BOARD_TOUCH_SWAP_XY           (0)
#define ESP_PANEL_BOARD_TOUCH_MIRROR_X          (0)
#define ESP_PANEL_BOARD_TOUCH_MIRROR_Y          (0)

#define ESP_PANEL_BOARD_TOUCH_RST_IO            (-1)
#define ESP_PANEL_BOARD_TOUCH_RST_LEVEL         (0)
#define ESP_PANEL_BOARD_TOUCH_INT_IO            (4)
#define ESP_PANEL_BOARD_TOUCH_INT_LEVEL         (0)

#endif // USE_TOUCH

// ---------------- BACKLIGHT -------------------

#define ESP_PANEL_BOARD_USE_BACKLIGHT           (1)

#if ESP_PANEL_BOARD_USE_BACKLIGHT

#define ESP_PANEL_BOARD_BACKLIGHT_TYPE          (ESP_PANEL_BACKLIGHT_TYPE_SWITCH_EXPANDER)
#define ESP_PANEL_BOARD_BACKLIGHT_IO            (2)
#define ESP_PANEL_BOARD_BACKLIGHT_ON_LEVEL      (1)
#define ESP_PANEL_BOARD_BACKLIGHT_IDLE_OFF      (0)

#endif

// ---------------- IO EXPANDER -------------------

#define ESP_PANEL_BOARD_USE_EXPANDER            (1)

#if ESP_PANEL_BOARD_USE_EXPANDER

#define ESP_PANEL_BOARD_EXPANDER_CHIP           CH422G
#define ESP_PANEL_BOARD_EXPANDER_SKIP_INIT_HOST     (0)
#define ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID        (0)
#define ESP_PANEL_BOARD_EXPANDER_I2C_CLK_HZ         (400 * 1000)
#define ESP_PANEL_BOARD_EXPANDER_I2C_SCL_PULLUP     (1)
#define ESP_PANEL_BOARD_EXPANDER_I2C_SDA_PULLUP     (1)
#define ESP_PANEL_BOARD_EXPANDER_I2C_IO_SCL         (9)
#define ESP_PANEL_BOARD_EXPANDER_I2C_IO_SDA         (8)
#define ESP_PANEL_BOARD_EXPANDER_I2C_ADDRESS        (0x20)

#endif

#define ESP_PANEL_BOARD_EXPANDER_POST_BEGIN_FUNCTION(p) \
    {  \
        auto board = static_cast<Board *>(p);  \
        auto expander = static_cast<esp_expander::CH422G*>(board->getIO_Expander()->getBase()); \
        expander->enableAllIO_Output(); \
        return true;    \
    }

#define ESP_PANEL_BOARD_LCD_PRE_BEGIN_FUNCTION(p) \
    {  \
        constexpr int LCD_RST = 3; \
        auto board = static_cast<Board *>(p);  \
        auto expander = board->getIO_Expander()->getBase(); \
        expander->digitalWrite(LCD_RST, 0); \
        vTaskDelay(pdMS_TO_TICKS(10)); \
        expander->digitalWrite(LCD_RST, 1); \
        vTaskDelay(pdMS_TO_TICKS(100)); \
        return true;    \
    }

#if ESP_PANEL_BOARD_USE_TOUCH
#define ESP_PANEL_BOARD_TOUCH_PRE_BEGIN_FUNCTION(p) \
    {  \
        constexpr gpio_num_t TP_INT = static_cast<gpio_num_t>(ESP_PANEL_BOARD_TOUCH_INT_IO); \
        constexpr int TP_RST = 1; \
        auto board = static_cast<Board *>(p);  \
        auto expander = board->getIO_Expander()->getBase(); \
        gpio_set_direction(TP_INT, GPIO_MODE_OUTPUT); \
        gpio_set_level(TP_INT, 0); \
        vTaskDelay(pdMS_TO_TICKS(10)); \
        expander->digitalWrite(TP_RST, 0); \
        vTaskDelay(pdMS_TO_TICKS(100)); \
        expander->digitalWrite(TP_RST, 1); \
        vTaskDelay(pdMS_TO_TICKS(200)); \
        gpio_reset_pin(TP_INT); \
        return true;    \
    }
#endif

#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MAJOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MINOR 0
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_PATCH 0

#endif
