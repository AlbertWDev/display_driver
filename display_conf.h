#pragma once

#define DISPLAY_DEVICE_ST7735   1
#define DISPLAY_DEVICE_ILI9341  2


#define DISPLAY_DEVICE_TYPE DISPLAY_DEVICE_ST7735

#if DISPLAY_DEVICE_TYPE == DISPLAY_DEVICE_ILI9341
    #define DISP_WIDTH  240
    #define DISP_HEIGHT 320
    #define DISP_DEPTH  16

    #define DISP_SPI_HOST   VSPI_HOST
    #define DISP_SPI_MAX_TRANSFER_SIZE  DISP_WIDTH * DISP_HEIGHT * 2 + 8
    #define DISP_TRANSACTION_QUEUE_SIZE 7
    #define DISP_PIN_CLK    18
    #define DISP_PIN_MISO   19
    #define DISP_PIN_MOSI   23
    #define DISP_PIN_CS     14
    #define DISP_PIN_DC     27
    #define DISP_PIN_RST    33
    #define DISP_PIN_BCKL   32

    #define DISP_SPI_CLOCK_SPEED 40000000
#elif DISPLAY_DEVICE_TYPE == DISPLAY_DEVICE_ST7735
    #define DISP_WIDTH  128
    #define DISP_HEIGHT 160
    #define DISP_DEPTH  16

    #define DISP_SPI_HOST   VSPI_HOST
    #define DISP_SPI_MAX_TRANSFER_SIZE  DISP_WIDTH * DISP_HEIGHT * 2 + 8
    #define DISP_TRANSACTION_QUEUE_SIZE 7
    #define DISP_PIN_CLK    18
    #define DISP_PIN_MISO   19
    #define DISP_PIN_MOSI   23
    #define DISP_PIN_CS     5
    #define DISP_PIN_DC     4
    #define DISP_PIN_RST    16
    #define DISP_PIN_BCKL   17

    // According to datasheet, minimum clock cycle for write operations is 66 ns (~ 15 MHz)
    #define DISP_SPI_CLOCK_SPEED 15000000
#endif
