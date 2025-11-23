#include "BMP3xx_platform.cpp"
#define FIFO_WATERMARK_FRAME_COUNT  UINT8_C(50)
#define FIFO_MAX_SIZE               UINT16_C(512)

int readData(){
    // Variable declarations below
    struct bmp3_dev dev;       // Device handle
    struct bmp3_spi_intf;      // Data struct
    int8_t rslt;               // Result code called by driver
    uint16_t settings_sel;     // Settings selection
    uint16_t settings_fifo;    //  FIFO selection 
    uint8_t index = 0;
    uint16_t fifo_length = 0;
    uint8_t fifo_data[FIFO_MAX_SIZE];
    struct bmp3_data fifo_p_t_data[FIFO_MAX_SIZE];
    struct bmp3_fifo_settings fifo_settings = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_fifo_data fifo = { 0 };
    struct bmp3_status status = { { 0 } };

    rslt = 



    return 1;
}