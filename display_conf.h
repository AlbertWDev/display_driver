#pragma once

#include "sdkconfig.h"

#define DISPLAY_DRIVER_ST7735   1
#define DISPLAY_DRIVER_ST7789VW 2
#define DISPLAY_DRIVER_ILI9341  3
#define DISPLAY_DRIVER_ILI9342C 4


#if defined(CONFIG_DISPLAY_DRIVER_ST7735)
    #define DISPLAY_DRIVER DISPLAY_DRIVER_ST7735

#elif defined(CONFIG_DISPLAY_DRIVER_ST7789VW)
    #define DISPLAY_DRIVER DISPLAY_DRIVER_ST7789VW

#elif defined(CONFIG_DISPLAY_DRIVER_ILI9341)
    #define DISPLAY_DRIVER DISPLAY_DRIVER_ILI9341

#elif defined(CONFIG_DISPLAY_DRIVER_ILI9342C)
    #define DISPLAY_DRIVER DISPLAY_DRIVER_ILI9342C

#else
    #error "Display driver type not set"
#endif


#define DISP_WIDTH  CONFIG_DISPLAY_WIDTH
#define DISP_HEIGHT CONFIG_DISPLAY_HEIGHT
#define DISP_DEPTH  CONFIG_DISPLAY_DEPTH


#if defined(CONFIG_DISPLAY_SPI_HOST_VSPI)
    #define DISP_SPI_HOST   VSPI_HOST
#elif defined(CONFIG_DISPLAY_SPI_HOST_HSPI)
    #define DISP_SPI_HOST   HSPI_HOST
#else
    #error "Display SPI host not set"
#endif

#define DISP_SPI_MAX_TRANSFER_SIZE  CONFIG_DISPLAY_SPI_MAX_TRANSFER_SIZE
#define DISP_SPI_QUEUE_SIZE         CONFIG_DISPLAY_SPI_QUEUE_SIZE
#define DISP_SPI_CLOCK_SPEED        CONFIG_DISPLAY_SPI_CLOCK_SPEED

#define DISP_PIN_MISO   CONFIG_DISPLAY_PIN_MISO
#define DISP_PIN_MOSI   CONFIG_DISPLAY_PIN_MOSI
#define DISP_PIN_CLK    CONFIG_DISPLAY_PIN_CLK
#define DISP_PIN_CS     CONFIG_DISPLAY_PIN_CS
#define DISP_PIN_DC     CONFIG_DISPLAY_PIN_DC
#define DISP_PIN_RST    CONFIG_DISPLAY_PIN_RST
#define DISP_PIN_BCKL   CONFIG_DISPLAY_PIN_BCKL
