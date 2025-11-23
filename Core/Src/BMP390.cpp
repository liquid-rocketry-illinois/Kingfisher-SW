#include "BMP3xx_platform.cpp"
#include <exception>
#define FIFO_WATERMARK_FRAME_COUNT  UINT8_C(50)
#define FIFO_MAX_SIZE               UINT16_C(512)

static struct bmp3_dev dev;       // Device handle


int initBMP390(){
    // Variable declarations below
    struct bmp3_spi_intf;      // SPI Interface
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

    try{
    rslt = bmp3_interface_init(&dev, BMP3_SPI_INTF);
    // TODO: Write a check result function
    rslt = bmp3_init(&dev);
    // Implement check function

    // Config FIFO settings to FIFO both pressure and temperature
    fifo_settings.mode = BMP3_DISABLE;
    
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
    settings.int_settings.latch = BMP3_ENABLE;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;

    settings.op_mode = BMP3_MODE_NORMAL;

    // Add check statements to each of the following
    rslt = bmp3_set_op_mode(&settings, &dev);
    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    return 1;

    }catch(std::exception e){
        return 0;
    }
}

void readBMP390(){

    static struct bmp3_data data;
    
}