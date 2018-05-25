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
//        https://github.com/nkolban/esp32-snippets/blob/master/rmt/fragments/rmt_simple.c
//        https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
//        DOUBLE BUFFERRING
//        http://gameprogrammingpatterns.com/double-buffer.html
//
//        BITMAP FONTS
//        http://jared.geek.nz/2014/jan/custom-fonts-for-microcontrollers
//
// APRIL 30, 2018
//
// ANKIT BHATNAGAR
// ANKIT.BHATNAGARINDIA@GMAIL.COM
//////////////////////////////////////////////////////////////////

#include "ESP32_WS2812B_PANEL.h"
#include "ESP32_UTIL.h"
#include "driver/rmt.h"
#include "soc/rmt_struct.h"
#include "soc/dport_reg.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_intr.h"
#include "homespun_font.h"

#define ESP32_WS28122B_PANEL_RMT_DATA_THRESHOLD_LIMIT   (32)

//INTERNAL VARIABLES
static bool s_debug;
static volatile bool s_frame_dirty;
static volatile bool s_frame_rendering;

//INTERRUPT RELATED
static intr_handle_t s_esp32_ws2812b_panel_rmt_int_handle;

//DISPLAY RELATED
static uint8_t s_esp32_ws2812b_panel_data_gpio;
static uint8_t s_esp32_ws2812b_panel_count_row;
static uint8_t s_esp32_ws2812b_panel_count_col;

//BUFFER RELATED
static uint8_t* s_esp32_ws2812b_panel_framebuffer_1;
static uint8_t* s_esp32_ws2812b_panel_framebuffer_2;
static uint8_t* s_esp32_ws2812b_panel_framebuffer_pointer[2];
static uint8_t s_esp32_ws2812b_panel_active_framebuffer_num;
static volatile uint32_t s_esp32_ws2812b_panel_framebuffer_data_counter;
static uint16_t s_esp32_ws2812b_panel_framebuffer_total_bytes;
static volatile bool s_esp32_ws2812b_panel_half_copy_pointer;

//INTERNAL FUNCTIONS
static void s_esp32_ws2812b_panel_rmt_isr(void* arg);
static void s_esp32_ws2812b_panel_reset(void);
static void s_esp32_ws2812b_panel_switch_framebuffer(void);
static void s_esp32_ws2812b_panel_copy_full_buffer(void);
static void s_esp32_ws2812b_panel_copy_half_buffer(bool at_beginning);
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

    if(rows > ESP32_WS2812B_PANEL_ROW_MAX)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : Max row length exceeded! Init FAILED\n");
        return;
    }

    s_esp32_ws2812b_panel_count_row = rows;
    s_esp32_ws2812b_panel_count_col = columns;
    s_esp32_ws2812b_panel_data_gpio = data_gpio;

    //ALLOCATE BUFFERS
    s_esp32_ws2812b_panel_framebuffer_1 = (uint8_t*)calloc((rows * columns * 3), 
                                                                sizeof(uint8_t));
    s_esp32_ws2812b_panel_framebuffer_2 = (uint8_t*)calloc((rows * columns * 3), 
                                                                sizeof(uint8_t));
    s_esp32_ws2812b_panel_framebuffer_pointer[0] = s_esp32_ws2812b_panel_framebuffer_1;
    s_esp32_ws2812b_panel_framebuffer_pointer[1] = s_esp32_ws2812b_panel_framebuffer_2;
    s_esp32_ws2812b_panel_active_framebuffer_num = 0;
    s_esp32_ws2812b_panel_framebuffer_data_counter = 0;
    s_esp32_ws2812b_panel_framebuffer_total_bytes = (3 * rows * columns);

    s_frame_rendering = false;
    s_esp32_ws2812b_panel_half_copy_pointer = true;
    s_frame_dirty = false;

    /*
    //RESET & START RMT PERIPHERAL
    //ENABLE CLOCK TO RMT
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

    //SETUP RMT GPIO
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_gpio], 2);
    gpio_matrix_out(data_gpio, RMT_SIG_OUT0_IDX + 0, 0, 0);
    gpio_set_direction(data_gpio, GPIO_MODE_OUTPUT);

    //INITIALIZE RMT PERIPHERAL
    RMT.apb_conf.fifo_mask = 1;
    RMT.apb_conf.mem_tx_wrap_en = 1;
    RMT.conf_ch[0].conf0.div_cnt = ESP32_WS28122B_PANEL_RMT_DIVIDER;
    RMT.conf_ch[0].conf0.mem_size = 1;
    RMT.conf_ch[0].conf0.carrier_en = 0;
    RMT.conf_ch[0].conf0.carrier_out_lv = 1;
    RMT.conf_ch[0].conf0.mem_pd = 0;

    RMT.conf_ch[0].conf1.rx_en = 0;
    RMT.conf_ch[0].conf1.mem_owner = 0;
    RMT.conf_ch[0].conf1.tx_conti_mode = 0;
    RMT.conf_ch[0].conf1.ref_always_on = 1;
    RMT.conf_ch[0].conf1.idle_out_en = 1;
    RMT.conf_ch[0].conf1.idle_out_lv = 0;
    */

    //SET RMT CONFIG
    //CAN USE PROVIDED RMT_DRIVER FOR THIS
    rmt_config_t rconfig;

    rconfig.rmt_mode = RMT_MODE_TX;
    rconfig.channel = 0;
    rconfig.gpio_num = data_gpio;
    rconfig.clk_div = ESP32_WS28122B_PANEL_RMT_DIVIDER;
    rconfig.mem_block_num = 1;
    
    rconfig.tx_config.loop_en = 0;
    rconfig.tx_config.idle_output_en = 0;
    rconfig.tx_config.idle_level = 0;

    rconfig.tx_config.carrier_en = 0;
    rconfig.tx_config.carrier_duty_percent = 50;
    rconfig.tx_config.carrier_freq_hz = 10000;
    rconfig.tx_config.carrier_level = 1;

    //INIT RMT
    ESP_ERROR_CHECK(rmt_config(&rconfig));

    //SETUP RMT INTERRUPT
    //CANNOT USE DRIVER FOR IT OR rmt_driver_install
    RMT.apb_conf.mem_tx_wrap_en = 1;
    RMT.tx_lim_ch[0].limit = ESP32_WS28122B_PANEL_RMT_DATA_THRESHOLD_LIMIT;
    RMT.int_ena.ch0_tx_thr_event = 1;
    RMT.int_ena.ch0_tx_end = 1;
    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, s_esp32_ws2812b_panel_rmt_isr, NULL, &s_esp32_ws2812b_panel_rmt_int_handle);

    ets_printf(ESP32_WS2812B_PANEL_TAG" : Initialized Panel %ux%u, data_gpio = %u\n",
                                                                        s_esp32_ws2812b_panel_count_row,
                                                                        s_esp32_ws2812b_panel_count_col,
                                                                        data_gpio);
    
    //RESET THE PANEL
    //s_esp32_ws2812b_panel_reset();
}

void ESP32_WS28182B_PANEL_RenderFrame(void)
{
    //RENDER THE INACTIVE FRAME BUFFER

    static uint8_t ctr = 0;

    //WAIT FOR ONGOING RENDERING JOB TO FINISH
    if(s_frame_rendering)
    {
        return;
    }

    ctr++;
    if(ctr == 3)
    {
        while(1){}
    }
    
    //RENDER ONLY IF FRAME DIRTY
    if(!s_frame_dirty)
    {
        return;
    }
    s_frame_dirty = false;

    ets_printf("start frame rendering %u\n", !s_esp32_ws2812b_panel_active_framebuffer_num);

    //CLEAR ALL EVENTS
    RMT.int_clr.ch0_tx_thr_event = 1;
    RMT.int_clr.ch0_tx_end = 1;

    //ENABLE RMT THR EVENT
    RMT.int_ena.ch0_tx_thr_event = 1;
    RMT.int_ena.ch0_tx_end = 1;

    //START NEW FRAME RENDERING
    s_frame_rendering = true;
    s_esp32_ws2812b_panel_half_copy_pointer = true;
    s_esp32_ws2812b_panel_framebuffer_data_counter = 0;
    
    //COPY FIRST FULL RMT DATA BUFFER
    s_esp32_ws2812b_panel_copy_full_buffer();
    
    //START RMT TRANSACTION
    RMT.conf_ch[0].conf1.mem_rd_rst = 1;
    RMT.conf_ch[0].conf1.tx_start = 1;
}

bool ESP32_WS2812B_PANEL_SetPixel(uint8_t x, 
                                    uint8_t y, 
                                    s_esp32_ws2812b_panel_color_t color)
{
    //SET PIXEL IN THE ACTIVE  FRAME BUFFER WITH SPECIFIED COLOR
    //G, R, B FORMAT

    if(x >= s_esp32_ws2812b_panel_count_col ||
    y >= s_esp32_ws2812b_panel_count_row)
    {
        //PIXEL OUT OF RANGE
        return false;
    }

    s_frame_dirty = true;

    //CONVERT CARTESIAN PIXEL TO WS2812B STRIP INDEX
    uint16_t index = s_esp32_ws2812b_panel_cartesian_pixel_to_strip_index(x, y);
    
    //STORE DATA IN FRAMEBUFFER AS G, R, B
    s_esp32_ws2812b_panel_framebuffer_pointer[s_esp32_ws2812b_panel_active_framebuffer_num][(index * 3) + 0] = color.g;
    s_esp32_ws2812b_panel_framebuffer_pointer[s_esp32_ws2812b_panel_active_framebuffer_num][(index * 3) + 1] = color.r;
    s_esp32_ws2812b_panel_framebuffer_pointer[s_esp32_ws2812b_panel_active_framebuffer_num][(index * 3) + 2] = color.b;

    if(s_debug)
    {
        //ets_printf(ESP32_WS2812B_PANEL_TAG" : set pixel (%u,%u). index = %u\n", x, y, index);
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

bool ESP32_WS2812B_PANEL_SetBoxFilled(uint8_t x, 
                                        uint8_t y, 
                                        uint8_t width, 
                                        uint8_t height,
                                        s_esp32_ws2812b_panel_color_t color)
{
    //SET BOX FILLED WITH SPECIFIED CORNER AND WIDTH AND HEIGHT

    bool retval = true;

    for(uint8_t i = 0; i < height; i++)
    {
        retval &= ESP32_WS2812B_PANEL_SetLineHorizontal(x, y + i, width, color);
    }

    return retval;
}

bool ESP32_WS2812B_PANEL_WriteStringXY(uint8_t x,
                                        uint8_t y,
                                        char* str,
                                        uint8_t str_len,
                                        s_esp32_ws2812b_panel_color_t color)
{
    //WRITE THE SPECIFIED STRING OF SPECIFIED LENGTH
    //AT THE SPECIFIED CORDINATES
    //FONT USED = 6(WIDTH) x 8(HEIGHT)


    if(x >= s_esp32_ws2812b_panel_count_col ||
        y >= s_esp32_ws2812b_panel_count_row)
    {
        //PIXEL OUT OF RANGE
        return false;
    }

    uint8_t x_start;
    uint8_t curr_char;

    x_start = x;
    for(uint8_t i = 0; i < str_len; i++)
    {
        
        curr_char = str[i];
        for(uint8_t j = 0; j < font_width[curr_char - 32]; j++)
        {
            if(font[curr_char - 32][j] & 1)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 0, color);
            }
            if(font[curr_char - 32][j] & 2)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 1, color);
            }
            if(font[curr_char - 32][j] & 4)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 2, color);
            }
            if(font[curr_char - 32][j] & 8)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 3, color);
            }
            if(font[curr_char - 32][j] & 16)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 4, color);
            }
            if(font[curr_char - 32][j] & 32)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 5, color);
            }
            if(font[curr_char - 32][j] & 64)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 6, color);
            }
            if(font[curr_char - 32][j] & 128)
            {
                ESP32_WS2812B_PANEL_SetPixel(x_start, y + 7, color);
            }
            x_start++;
        }
        x_start++;
    }
    return true;
}

bool ESP32_WS2812B_PANEL_WriteStringJustified(esp32_ws2812b_panel_justify_t justify,
                                                char* str,
                                                uint8_t str_len,
                                                s_esp32_ws2812b_panel_color_t color)
{
    //WRITE SPECIFIED STRING WITH SPECIFIED JUSTIFICATION RELATIVE
    //TO THE PANEL

    if(justify >= ESP32_WS2812B_PANEL_JUSTIFY_MAX)
    {
        //WRONG JUSTIFICATION
        return false;
    }

    //CALCULATE STARTING x,y BASED ON STRING AND JUSTIFICATION
    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t string_width = 0;
    y = 0;

    for(uint8_t i = 0; i < str_len; i++)
    {
        string_width += font_width[str[i] - 32] + 1;
    }
    string_width--;

    if(string_width > s_esp32_ws2812b_panel_count_col)
    {
        //STRING WONT FIT
        return false;
    }

    switch(justify)
    {
        case ESP32_WS2812B_PANEL_JUSTIFY_LEFT:
            x = 0;
            break;
        
        case ESP32_WS2812B_PANEL_JUSTIFY_CENTRE:
            x = (s_esp32_ws2812b_panel_count_col - string_width ) / 2;
            break;
        
        case ESP32_WS2812B_PANEL_JUSTIFY_RIGHT:
            x = s_esp32_ws2812b_panel_count_col - string_width;
            break;
        
        default:
            break;
    }
    return ESP32_WS2812B_PANEL_WriteStringXY(x, y, str, str_len, color);
}

bool ESP32_WS2812B_PANEL_Clear(void)
{
    //CLEAR PANEL

    s_esp32_ws2812b_panel_color_t black = {0, 0, 0};
    for(uint8_t x = 0; x < s_esp32_ws2812b_panel_count_col; x++)
    {
        for(uint8_t y = 0; y < s_esp32_ws2812b_panel_count_row; y++)
        {
            ESP32_WS2812B_PANEL_SetPixel(x, y, black);
        }
    }

    if(s_debug)
    {
        ets_printf(ESP32_WS2812B_PANEL_TAG" : clear\n");
    }
    return true;
}

static void s_esp32_ws2812b_panel_rmt_isr(void* arg)
{
    //RMT INTERRUPT HANDLER

    if(RMT.int_st.ch0_tx_end)
    {
        //TX END EVENT

        //CLEAR EVENT
        RMT.int_clr.ch0_tx_end = 1;

        //END OF FRAMEBUFFFER. SWITCH
        s_esp32_ws2812b_panel_switch_framebuffer();

        ets_printf("RMT TX END\n");
        return;
    }

    if(RMT.int_st.ch0_tx_thr_event)
    {
        //TX THRESHOLD EVENT
        //32 OF 64 RMT_32 ITEMS IN RMT MEM BLOCK 1 DONE

        //CLEAR EVENT
        RMT.int_clr.ch0_tx_thr_event = 1;

        //DO HALF COPY
        s_esp32_ws2812b_panel_copy_half_buffer(s_esp32_ws2812b_panel_half_copy_pointer);
        s_esp32_ws2812b_panel_half_copy_pointer = !s_esp32_ws2812b_panel_half_copy_pointer;

        ets_printf("RMT THR\n");
        return;
    }
}

static void s_esp32_ws2812b_panel_reset(void)
{
    //RESET WS2812B PANEL BY SENDING RESET PULSE

    gpio_set_level(s_esp32_ws2812b_panel_data_gpio, 0);
    ESP32_UTIL_DelayBlockingMs(50);
    gpio_set_level(s_esp32_ws2812b_panel_data_gpio, 1);

    ets_printf(ESP32_WS2812B_PANEL_TAG" : RESET\n");
}

static void s_esp32_ws2812b_panel_switch_framebuffer(void)
{
    //SWITCH FRAMBUFFER

    s_esp32_ws2812b_panel_active_framebuffer_num = !s_esp32_ws2812b_panel_active_framebuffer_num;
}

static void s_esp32_ws2812b_panel_copy_full_buffer(void)
{
    //FILL RMT DATA SPACE WITH FULL DATA (1 MEM BLOCK = 64 RMT_32 ITEMS)
    //WILL ONLY BE CALLED AT START OF FRAME RENDERING
    //RMT 1 MEMBLOCK CAN HOLD 64 RMT_32 ITEMS = 64 WS2812 BITS = 64 WS2812 BYTES
    //IN RMT_32 ITEM, DATA IS SENT HIGH FIRST, IE LEVEL1 IS SENT FIRST

    esp32_ws2812b_panel_rmt_bit_u rmt_bit;
    uint16_t counter = 0;
    uint8_t val;
    uint8_t data_end = 0;

    //CHECK IF TOTAL WS28182 PIXEL DATA CAN FIT IN SINGLE MEM BLOCK WITH NO WRAP
    //AROUND. IF SO ADD TERMINATING 0 ASLO
    if((s_esp32_ws2812b_panel_framebuffer_total_bytes * 8) < 64)
    {
        data_end = s_esp32_ws2812b_panel_framebuffer_total_bytes;
    }
    else
    {
        data_end = 8;
    }

    ets_printf("full copy till %u\n", data_end);

    for(uint8_t i = 0; i < data_end; i++)
    {   
        val = s_esp32_ws2812b_panel_framebuffer_pointer[!s_esp32_ws2812b_panel_active_framebuffer_num][i];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(val & 0x70)
            {
                rmt_bit.components.level1 = 1;
                rmt_bit.components.duration1 = ESP32_WS28122B_PANEL_PULSE_T1H;
                rmt_bit.components.level0 = 0;
                rmt_bit.components.duration0 = ESP32_WS28122B_PANEL_PULSE_T1L;
                //ets_printf("bit 1: %u %u %u %u\n", rmt_bit.components.level1, rmt_bit.components.duration1, rmt_bit.components.level0, rmt_bit.components.duration0);
                RMTMEM.chan[0].data32[counter].val = rmt_bit.val;
            }
            else
            {
                rmt_bit.components.level1 = 1;
                rmt_bit.components.duration1 = ESP32_WS28122B_PANEL_PULSE_T0H;
                rmt_bit.components.level0 = 0;
                rmt_bit.components.duration0 = ESP32_WS28122B_PANEL_PULSE_T0L;
                //ets_printf("bit 0: %u %u %u %u\n", rmt_bit.components.level1, rmt_bit.components.duration1, rmt_bit.components.level0, rmt_bit.components.duration0);
                RMTMEM.chan[0].data32[counter].val = rmt_bit.val;
            }
            ets_printf("%u\n", rmt_bit.val);
            val = val << 1;
            counter++;
        }
        s_esp32_ws2812b_panel_framebuffer_data_counter++;
    }
}

static void s_esp32_ws2812b_panel_copy_half_buffer(bool at_beginning)
{
    //COPY HALF BUFFER TO RMT DATA SPACE (32 RMT_32 BITS)
    //WEATHER TO COPY FROM RMT DATA SPACE BEGENNING OR FROM
    //THE MIDDLE DEPENDS ON THE BOOL ARGUMENT
    //RMT 1 MEMBLOCK CAN HOLD 64 RMT_32 ITEMS = 64 WS2812 BITS = 64 WS2812 BYTES
    //IN RMT_32 ITEM, DATA IS SENT HIGH FIRST, IE LEVEL1 IS SENT FIRST

    esp32_ws2812b_panel_rmt_bit_u rmt_bit;
    uint16_t counter = 0;
    uint8_t starting;
    uint8_t val;

    starting = ((at_beginning) ? 0 : 32);

    //CHECK FOR TX DATA ENDING
    if(s_esp32_ws2812b_panel_framebuffer_data_counter == s_esp32_ws2812b_panel_framebuffer_total_bytes)
    {
        //FRAMEBUFFER DATA END HAS REACHED
        //INSERT 0 TO END RMT T
        RMTMEM.chan[0].data32[starting].val = 0;
    }

    ets_printf("half copy from %u till %u\n", starting, starting + 4);

    for(uint8_t i = 0; i < (8/2); i++)
    {   
        val = s_esp32_ws2812b_panel_framebuffer_pointer[!s_esp32_ws2812b_panel_active_framebuffer_num][s_esp32_ws2812b_panel_framebuffer_data_counter];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(val & 0x70)
            {
                rmt_bit.components.level1 = 1;
                rmt_bit.components.duration1 = ESP32_WS28122B_PANEL_PULSE_T1H;
                rmt_bit.components.level0 = 0;
                rmt_bit.components.duration0 = ESP32_WS28122B_PANEL_PULSE_T1L;
                RMTMEM.chan[0].data32[starting + counter].val = rmt_bit.val;
            }
            else
            {
                rmt_bit.components.level1 = 1;
                rmt_bit.components.duration1 = ESP32_WS28122B_PANEL_PULSE_T0H;
                rmt_bit.components.level0 = 0;
                rmt_bit.components.duration0 = ESP32_WS28122B_PANEL_PULSE_T0L;
                RMTMEM.chan[0].data32[starting + counter].val = rmt_bit.val;
            }
            ets_printf("%u\n", rmt_bit.val);
            val = val << 1;
            counter++;
        }
        s_esp32_ws2812b_panel_framebuffer_data_counter++;
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