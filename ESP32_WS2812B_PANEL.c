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

//RMT RELATED
static intr_handle_t s_esp32_ws2812b_panel_interrupt_handle;
uint16_t s_esp32_ws2812b_panel_blocksize;

//DISPLAY RELATED
uint8_t s_esp32_ws2812b_panel_count_row;
uint8_t s_esp32_ws2812b_panel_count_col;

//BUFFER RELATED
static volatile uint8_t s_esp32_ws2812b_panel_next_coloumn;
rmt_item32_t* s_esp32_ws2812b_panel_buffer;
s_esp32_ws2812b_panel_color_t* s_esp32_ws2812b_panel_rgb_buffer;


//INTERNAL FUNCTIONS
static void s_esp32_ws2812b_panel_send_column(uint8_t col);
static void s_esp32_ws2812b_panel_rmt_isr(void* pArg);


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

    s_ongoing = false;

    //ALLOCATE BUFFERS
    s_esp32_ws2812b_panel_rgb_buffer = (s_esp32_ws2812b_panel_color_t*)calloc((rows * columns), 
                                                                sizeof(s_esp32_ws2812b_panel_color_t));
    s_esp32_ws2812b_panel_buffer = (rmt_item32_t*)calloc((rows * 24), sizeof(rmt_item32_t));

    //CONFIGURE ESP32 RMT
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL_0;
    config.gpio_num = data_gpio;

    s_esp32_ws2812b_panel_blocksize = ((rows * 24) / 64) + 1;

    config.mem_block_num = s_esp32_ws2812b_panel_blocksize;
    config.tx_config.loop_en = 0;
    config.tx_config.carrier_en = 0;
    config.tx_config.idle_output_en = 1;
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

bool ESP32_WS2812B_PANEL_SetPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
    //SET PIXEL IN THE INTERNAL BUFFER

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

    uint32_t counter = (x * s_esp32_ws2812b_panel_count_row) + (y);
    s_esp32_ws2812b_panel_rgb_buffer[counter].r = r; 
    s_esp32_ws2812b_panel_rgb_buffer[counter].g = g;
    s_esp32_ws2812b_panel_rgb_buffer[counter].b = b;

    return true;
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

    return true;
}

void ESP32_WS28182B_PANEL_Refresh(void)
{
    //REFRESH DISPLAY USING THE INTERNAL RGB BUFFER
    //SEND THE WHOLE BUFFER OUT

    //BLOCK TILL LAST OPERATION COMPLETES
    while(s_ongoing != false){};

    //START OPERATION
    s_esp32_ws2812b_panel_next_coloumn = 0;
    while(s_esp32_ws2812b_panel_next_coloumn < s_esp32_ws2812b_panel_count_col)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG"writing column %u\n", s_esp32_ws2812b_panel_next_coloumn);
        s_esp32_ws2812b_panel_send_column(s_esp32_ws2812b_panel_next_coloumn);
        while(rmt_wait_tx_done(0, 1000) != ESP_OK){};
        s_esp32_ws2812b_panel_next_coloumn++;
    }

    if(s_debug)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : Refresh\n");
    }
}

static void s_esp32_ws2812b_panel_send_column(uint8_t col)
{
    //SEND THE BUFFER RELATED TO SPECIFIED COLUMN OUT

    //FILL RMT BUFFER WITH COL DATA
    for(uint16_t i = 0; i < s_esp32_ws2812b_panel_count_row; i++)
    {
        uint16_t counter = (i * 24);
        uint8_t pixel_r = s_esp32_ws2812b_panel_rgb_buffer[(s_esp32_ws2812b_panel_count_row * col) + (i)].r;
        uint8_t pixel_g = s_esp32_ws2812b_panel_rgb_buffer[(s_esp32_ws2812b_panel_count_row * col) + (i)].g;
        uint8_t pixel_b = s_esp32_ws2812b_panel_rgb_buffer[(s_esp32_ws2812b_panel_count_row * col) + (i)].b;
        for(int8_t j = 0; j <= 7; j++)
        {
            if(pixel_b & (1 << (7-j)))
            {
                //1
                s_esp32_ws2812b_panel_buffer[counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[counter + j].duration0 = 2;
                s_esp32_ws2812b_panel_buffer[counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[counter + j].duration1 = 1;
            }
            else
            {
                //0
                s_esp32_ws2812b_panel_buffer[counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[counter + j].duration0 = 1;
                s_esp32_ws2812b_panel_buffer[counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[counter + j].duration1 = 2;
            }

            if(pixel_r & (1 << (7-j)))
            {
                //1
                s_esp32_ws2812b_panel_buffer[8 + counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].duration0 = 2;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].duration1 = 1;
            }
            else
            {
                //0
                s_esp32_ws2812b_panel_buffer[8 + counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].duration0 = 1;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[8 + counter + j].duration1 = 2;
            }

            if(pixel_g & (1 << (7-j)))
            {
                //1
                s_esp32_ws2812b_panel_buffer[16 + counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].duration0 = 2;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].duration1 = 1;
            }
            else
            {
                //0
                s_esp32_ws2812b_panel_buffer[16 + counter + j].level0 = 1;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].duration0 = 1;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].level1 = 0;
                s_esp32_ws2812b_panel_buffer[16 + counter + j].duration1 = 2;
            }
        }
    }

    //SEND DATA OUT
    rmt_write_items(0, s_esp32_ws2812b_panel_buffer, (24 * s_esp32_ws2812b_panel_count_row), 1);
}

/*static void s_esp32_ws2812b_panel_rmt_isr(void* pArg)
{
    //RMT TX INTERRUPT
    ets_printf("IABBAB!!!!!!\n");
    if(RMT.int_st.ch0_tx_thr_event)
    {
        //CLEAR INTERRUPT EVENT
        RMT.int_st.ch0_tx_thr_event = 1;
        //SET NEXT COLUMN FOR TRANSFER
        s_esp32_ws2812b_panel_next_coloumn++;
        ets_printf("ISR!!!!!!!\n");
    }
}*/