//////////////////////////////////////////////////////////////////
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
// APRIL 30, 2018
//
// ANKIT BHATNAGAR
// ANKIT.BHATNAGARINDIA@GMAIL.COM
//////////////////////////////////////////////////////////////////

#include "ESP32_WS2812B_PANEL.h"
#include "driver/rmt.h"
#include "esp_intr.h"

//INTERNAL VARIABLES
static bool s_debug;
static bool s_ongoing;

//DISPLAY RELATED
static uint8_t s_esp32_ws2812b_panel_count_row;
static uint8_t s_esp32_ws2812b_panel_count_col;

//BUFFER RELATED
static s_esp32_ws2812b_panel_color_t* s_esp32_ws2812b_panel_rgb_buffer;
static s_esp32_ws2812b_panel_color_t* s_esp32_ws2812b_next_pixel_pointer;
static rmt_item32_t s_exp32_ws2812b_rmt_pixel[24];

//INTERNAL FUNCTIONS
static void s_esp32_ws2812b_panel_send_next_pixel(void);
static uint16_t s_esp32_ws2812b_panel_cartesian_pixel_to_strip_index(uint8_t x, uint8_t y);

void ESP32_WS2812B_PANEL_SetDebug(bool enable)
{
	//SET MODULE DEBUG

	s_debug = enable;
}

void ESP32_WS2812B_PANEL_Initialize(uint8_t rows, uint8_t columns, uint8_t data_gpio)
{
    //INTIALIZE ESP32 WS2812B
    //RMT BASE CLOCK = 80MHZ
    //CLOCK DIVIDER = 28
    //WS2812B PIXEL = 3 WS2812B BYTES = 24 WS2812B BITS
    //1 WS2812B BIT = 2 RMT DATA ITEM = 1 RMT_ITEM32_T
    //MAX LIMIT OF RMT_ITEM32_T = 8 * 64 = 512 => 512 WS2812B BITS = 64 WS2812B BYTES ~ 31 WS2812B PIXELS
    //SINCE WE HANDLE PANEL COLUMN WISE, COLUMN <= 31

    if(rows > ESP32_WS2812B_PANEL_ROW_MAX)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : Max row length exceeded! Init FAILED\n");
        return;
    }

    s_esp32_ws2812b_panel_count_row = rows;
    s_esp32_ws2812b_panel_count_col = columns;

    //ALLOCATE BUFFERS
    s_esp32_ws2812b_panel_rgb_buffer = (s_esp32_ws2812b_panel_color_t*)calloc((rows * columns), 
                                                                sizeof(s_esp32_ws2812b_panel_color_t));

    s_esp32_ws2812b_next_pixel_pointer = &s_esp32_ws2812b_panel_rgb_buffer[0];
    s_ongoing = false;

    //CONFIGURE ESP32 RMT
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL_0;
    config.gpio_num = data_gpio;

    uint16_t s_esp32_ws2812b_panel_blocksize = ((rows * 24) / 64) + 1;

    config.mem_block_num = s_esp32_ws2812b_panel_blocksize;
    config.tx_config.loop_en = 0;
    config.tx_config.carrier_en = 0;
    config.tx_config.idle_output_en = 0;
    config.tx_config.idle_level = 0;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_freq_hz = 10000;
    config.tx_config.carrier_level = 1;
    config.clk_div = 28;
    ets_printf(ESP32_WS2812B_PANEL_TAG" : RMT initialized\n");

    //START ESP32 RMT
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ets_printf(ESP32_WS2812B_PANEL_TAG" : RMT started\n");

    ets_printf(ESP32_WS2812B_PANEL_TAG" : Initialized. Rows = %u, Columns = %u, DIN gpio = %u\n",
                                                                        s_esp32_ws2812b_panel_count_row,
                                                                        s_esp32_ws2812b_panel_count_col,
                                                                        data_gpio);
}

bool ESP32_WS2812B_PANEL_SetPixel(uint8_t x, 
                                    uint8_t y, 
                                    s_esp32_ws2812b_panel_color_t color)
{
    //SET PIXEL IN THE INTERNAL BUFFER WITH SPECIFIED COLOR

    if(s_ongoing)
    {
        //IN MIDDLE OF REFRESHING
        //DONT DISTURB BUFFER NOW
        return false;
    }

    if(x >= s_esp32_ws2812b_panel_count_col ||
        y >= s_esp32_ws2812b_panel_count_row)
    {
        //PIXEL OUT OF RANGE
        return false;
    }

    //COMVERT CARTESIAN PIXEL TO WS2812B STRIP INDEX
    uint16_t index = s_esp32_ws2812b_panel_cartesian_pixel_to_strip_index(x, y);

    ets_printf("index = %u\n", index);
    s_esp32_ws2812b_panel_rgb_buffer[index].r = color.r; 
    s_esp32_ws2812b_panel_rgb_buffer[index].g = color.g;
    s_esp32_ws2812b_panel_rgb_buffer[index].b = color.b;

    if(s_debug)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : set pixel (%u,%u)\n", x, y);
    }
    return true;
}

bool ESP32_WS2812B_PANEL_SetLineHorizontal(uint8_t x, 
                                            uint8_t y, 
                                            uint8_t len, 
                                            s_esp32_ws2812b_panel_color_t color)
{
    //HORIZONTAL LINE STARTING FROM SPECIFIED CORDINATES
    //OF SPECIFIED LENGTH AND COLOR

    if(s_ongoing)
    {
        //IN MIDDLE OF REFRESHING
        //DONT DISTURB BUFFER NOW
        return false;
    }

    if(x >= s_esp32_ws2812b_panel_count_col ||
        y >= s_esp32_ws2812b_panel_count_row ||
        (x + len) > s_esp32_ws2812b_panel_count_col)
    {
        //PIXEL OUT OF RANGE
        return false;
    }

    for(uint8_t i = 0; i < len; i++)
    {
        ESP32_WS2812B_PANEL_SetPixel(x + i, y, color);
    }
    return true;
}

bool ESP32_WS2812B_PANEL_SetLineVertical(uint8_t x, 
                                            uint8_t y, 
                                            uint8_t len, 
                                            s_esp32_ws2812b_panel_color_t color)
{
    //VERTICAL LINE STARTING FROM SPECIFIED CORDINATES
    //OF SPECIFIED LENGTH AND COLOR

    if(s_ongoing)
    {
        //IN MIDDLE OF REFRESHING
        //DONT DISTURB BUFFER NOW
        return false;
    }

    if(x >= s_esp32_ws2812b_panel_count_col ||
        y >= s_esp32_ws2812b_panel_count_row ||
        (y + len) > s_esp32_ws2812b_panel_count_row)
    {
        //PIXEL OUT OF RANGE
        return false;
    }

    for(uint8_t i = 0; i < len; i++)
    {
        ESP32_WS2812B_PANEL_SetPixel(x, y + i, color);
    }
    return true;
}

bool ESP32_WS2812B_PANEL_SetBoxOutline(uint8_t x, 
                                        uint8_t y, 
                                        uint8_t width, 
                                        uint8_t height,
                                        s_esp32_ws2812b_panel_color_t color)
{
    //SET BOX OUTLINE WITH SPECIFIED CORNER AND WIDTH AND HEIGHT

    bool retval = true;

    retval &= ESP32_WS2812B_PANEL_SetLineVertical(x, y, height, color);
    retval &= ESP32_WS2812B_PANEL_SetLineVertical(x + width - 1, y, height, color);
    retval &= ESP32_WS2812B_PANEL_SetLineHorizontal(x, y, width, color);
    retval &= ESP32_WS2812B_PANEL_SetLineHorizontal(x, y + height - 1, width, color);

    return retval;
}

bool ESP32_WS2812B_PANEL_Clear(void)
{
    //CLEAR PANEL

    if(s_ongoing)
    {
        //IN MIDDLE OF REFRESHING
        //DONT DISTURB BUFFER NOW
        return false;
    }

    //CLEAR PANEL
    s_esp32_ws2812b_panel_color_t black = {0, 0, 0};
    for(uint16_t i = 0; i < (s_esp32_ws2812b_panel_count_col * s_esp32_ws2812b_panel_count_row); i++)
    {
        s_esp32_ws2812b_panel_rgb_buffer[i].r = 0;
        s_esp32_ws2812b_panel_rgb_buffer[i].g = 0;
        s_esp32_ws2812b_panel_rgb_buffer[i].b = 0;
    }

    if(s_debug)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : clear\n");
    }

    return true;
}

void ESP32_WS28182B_PANEL_Refresh(void)
{
    //REFRESH DISPLAY USING THE INTERNAL RGB BUFFER
    //SEND THE WHOLE BUFFER OUT

    //BLOCK TILL LAST OPERATION COMPLETES
    while(s_ongoing){};
    
    s_ongoing = true;

    //START OPERATION
    //SEND OUT DATA RGB PIXEL BY PIXEL
    for(uint16_t m = 0; m < (s_esp32_ws2812b_panel_count_col * s_esp32_ws2812b_panel_count_row); m++)
    {
        s_esp32_ws2812b_panel_send_next_pixel();
    }

    s_ongoing = false;

    if(s_debug)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : Refresh\n");
    }
}

static void s_esp32_ws2812b_panel_send_next_pixel(void)
{
    //SEND NEXT PIXEL OUT USING RMT PERIPHERAL

    static uint32_t counter = 0;
    uint8_t pixel_r = s_esp32_ws2812b_next_pixel_pointer->r;
    uint8_t pixel_g = s_esp32_ws2812b_next_pixel_pointer->g;
    uint8_t pixel_b = s_esp32_ws2812b_next_pixel_pointer->b;  

    //INITIALIZE RMT PIXEL WITH DEFAULT VALUES
    for(uint8_t i = 0; i < 24; i++)
    {
        s_exp32_ws2812b_rmt_pixel[i].level0 = 1;
        s_exp32_ws2812b_rmt_pixel[i].level1 = 0;
        //MAKE BY DEFAULT ALL WS2812B BITS AS 0
        s_exp32_ws2812b_rmt_pixel[i].duration0 = 1;
        s_exp32_ws2812b_rmt_pixel[i].duration1 = 2;
    }

    //NOW FILL IN ALL 1 BITS IN WS2812B BITS
    //GREEN
    if(pixel_g & 128)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[0].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[0].duration1 = 1;
    }
    if(pixel_g & 64)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[1].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[1].duration1 = 1;
    }
    if(pixel_g & 32)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[2].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[2].duration1 = 1;
    }
    if(pixel_g & 16)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[3].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[3].duration1 = 1;
    }
    if(pixel_g & 8)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[4].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[4].duration1 = 1;
    }
    if(pixel_g & 4)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[5].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[5].duration1 = 1;
    }
    if(pixel_g & 2)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[6].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[6].duration1 = 1;
    }
    if(pixel_g & 1)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[7].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[7].duration1 = 1;
    }

    //RED
    if(pixel_r & 128)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[8].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[8].duration1 = 1;
    }
    if(pixel_r & 64)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[9].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[9].duration1 = 1;
    }
    if(pixel_r & 32)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[10].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[10].duration1 = 1;
    }
    if(pixel_r & 16)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[11].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[11].duration1 = 1;
    }
    if(pixel_r & 8)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[12].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[12].duration1 = 1;
    }
    if(pixel_r & 4)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[13].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[13].duration1 = 1;
    }
    if(pixel_r & 2)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[14].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[14].duration1 = 1;
    }
    if(pixel_r & 1)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[15].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[15].duration1 = 1;
    }

    //BLUE
    if(pixel_b & 128)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[16].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[16].duration1 = 1;
    }
    if(pixel_b & 64)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[17].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[17].duration1 = 1;
    }
    if(pixel_b & 32)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[18].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[18].duration1 = 1;
    }
    if(pixel_b & 16)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[19].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[19].duration1 = 1;
    }
    if(pixel_b & 8)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[20].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[20].duration1 = 1;
    }
    if(pixel_b & 4)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[21].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[21].duration1 = 1;
    }
    if(pixel_b & 2)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[22].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[22].duration1 = 1;
    }
    if(pixel_b & 1)
    {
        //1
        s_exp32_ws2812b_rmt_pixel[23].duration0 = 2;
        s_exp32_ws2812b_rmt_pixel[23].duration1 = 1;
    }

    //SEND OUT THE PIXEL IN BLOCKING MODE
    rmt_write_items(0, s_exp32_ws2812b_rmt_pixel, 24, false);

    //MODIFY COUNTERS FOR NEXT WRITE
    s_esp32_ws2812b_next_pixel_pointer++;
    counter++;
    if(counter >= (s_esp32_ws2812b_panel_count_row * s_esp32_ws2812b_panel_count_col))
    {
        //ALL PIXELS SENT
        //RESET PIX_POINTER
        s_esp32_ws2812b_next_pixel_pointer = &s_esp32_ws2812b_panel_rgb_buffer[0];
    }
}

static uint16_t s_esp32_ws2812b_panel_cartesian_pixel_to_strip_index(uint8_t x, uint8_t y)
{
    //CONVERT CARTESION COORDINATE TO WS2812B STRIP INDEX

    uint16_t retval;

    if((x % 2) == 0)
    {
        retval = (s_esp32_ws2812b_panel_count_row * x) + y;
    }
    else
    {
        retval = (s_esp32_ws2812b_panel_count_row * x) + (s_esp32_ws2812b_panel_count_row - y) - 1;
    }
    return retval;
}