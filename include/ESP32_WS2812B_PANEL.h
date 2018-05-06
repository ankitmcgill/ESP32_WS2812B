//////////////////////////////////////////////////////////////////
// ESP32 - WS2812B PANEL
//
// WS2812B RGB LED PANEL DRIVER
// USES ESP32 RMT (REMOTE CONTROL) PERIPHERAL TO GENERATE NRZ SIGNAL
//
// ONLY SUPPORTS 1 CHANNEL WITH MULTIPLE MEMORY BLOCKS
// SUPPORT M(ROWS) x N(COLUMNS) PANEL
// PANEL UPDATING DONE COLUMN-WISE STARTING TOP LEFT
//
// NOTE : FROM ESP-IDF MANUAL
//        http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/rmt.html?highlight=rmt
//          
//        NEIL KOLBAN ESP32 BOOK
//        SECTION = RMT
//
//        https://github.com/nkolban/esp32-snippets/blob/master/rmt/fragments/rmt_simple.c
//
//        https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
//        BITMAP FONTS
//        http://jared.geek.nz/2014/jan/custom-fonts-for-microcontrollers
//
// APRIL 30, 2018
//
// ANKIT BHATNAGAR
// ANKIT.BHATNAGARINDIA@GMAIL.COM
//////////////////////////////////////////////////////////////////

#ifndef _ESP32_WS2812B_PANEL_
#define _ESP32_WS2812B_PANEL_

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#define ESP32_WS2812B_PANEL_TAG         "ESP32:WS2812B_PANEL"
#define ESP32_WS2812B_PANEL_ROW_MAX     (31)

typedef enum
{
    ESP32_WS2812B_PANEL_JUSTIFY_LEFT = 0,
    ESP32_WS2812B_PANEL_JUSTIFY_CENTRE,
    ESP32_WS2812B_PANEL_JUSTIFY_RIGHT,
    ESP32_WS2812B_PANEL_JUSTIFY_MAX
}esp32_ws2812b_panel_justify_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}s_esp32_ws2812b_panel_color_t;

void ESP32_WS2812B_PANEL_SetDebug(bool enable);
void ESP32_WS2812B_PANEL_Initialize(uint8_t rows, uint8_t columns, uint8_t data_gpio);

bool ESP32_WS2812B_PANEL_SetPixel(uint8_t x, 
                                    uint8_t y, 
                                    s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_SetLineHorizontal(uint8_t x, 
                                            uint8_t y, 
                                            uint8_t len, 
                                            s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_SetLineVertical(uint8_t x, 
                                            uint8_t y, 
                                            uint8_t len, 
                                            s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_SetBoxOutline(uint8_t x, 
                                        uint8_t y, 
                                        uint8_t width, 
                                        uint8_t height,
                                        s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_SetBoxFilled(uint8_t x, 
                                        uint8_t y, 
                                        uint8_t width, 
                                        uint8_t height,
                                        s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_WriteStringXY(uint8_t x,
                                        uint8_t y,
                                        char* str,
                                        uint8_t str_len,
                                        s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_WriteStringJustified(esp32_ws2812b_panel_justify_t justify,
                                                char* str,
                                                uint8_t str_len,
                                                s_esp32_ws2812b_panel_color_t color);
bool ESP32_WS2812B_PANEL_Clear(void);
void ESP32_WS28182B_PANEL_Refresh(void);

#endif