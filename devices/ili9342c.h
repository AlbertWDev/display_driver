#pragma once

#include "esp_system.h"
#include "display_conf.h"

#define DISP_X_OFFSET 0
#define DISP_Y_OFFSET 0


#define DISP_BCKL_ENABLE    1
#define DISP_BCKL_DISABLE   0

// No operation
#define DISP_CMD_NOP        0x00
// Software reset
#define DISP_CMD_SWRESET    0x01
// Read Display ID
#define DISP_CMD_RDDID		0x04
// Read Display Status
#define DISP_CMD_RDDST		0x09
// Read Display Power Mode
#define DISP_CMD_RDDPM      0x0A
// Read Display MADCTL
#define DISP_CMD_RDDMADCTL  0x0B
// Read Display Pixel Format
#define DISP_CMD_RDDCOLMOD  0x0C
// Read Display Image Format
#define DISP_CMD_RDDIM      0x0D
// Read Display Signal Mode
#define DISP_CMD_RDDSM      0x0E
// Read Display Self-Diagnostic Result
#define DISP_CMD_RDDSDR     0x0F
// Enter Sleep Mode
#define DISP_CMD_SLPIN      0x10
// Sleep Out
#define DISP_CMD_SLPOUT     0x11
// Partial Mode ON
#define DISP_CMD_PTLON      0x12
// Normal Display Mode ON
#define DISP_CMD_NORON      0x13
// Display Inversion OFF
#define DISP_CMD_INVOFF     0x20
// Display Inversion ON
#define DISP_CMD_INVON      0x21
// Gamma Set
#define DISP_CMD_GAMSET     0x26
// Display OFF
#define DISP_CMD_DISPOFF    0x28
// Display ON
#define DISP_CMD_DISPON     0x29
// Column Address Set
#define DISP_CMD_CASET      0x2A
// Page (Row) Address Set
#define DISP_CMD_RASET      0x2B
// Memory Write
#define DISP_CMD_RAMWR      0x2C
// Color Set
#define DISP_CMD_RGBSET     0x2D
// Memory Read
#define DISP_CMD_RAMRD      0x2E
// Partial Area
#define DISP_CMD_PTLAR      0x30
// Vertical Scrolling Definition
#define DISP_CMD_VSCRDEF    0x33
// Tearing Effect Line OFF
#define DISP_CMD_TEOFF      0x34
// Tearing Effect Line ON
#define DISP_CMD_TEON       0x35
// Memory Access Control
#define DISP_CMD_MADCTL     0x36
    /// Mirror Y (Row Access Order)
    #define MADCTL_MY       0x80
    /// Mirror X (Column Access Order)
    #define MADCTL_MX       0x40
    /// Row/Column Exchange
    #define MADCTL_MV       0x20
    /// Vertical Refresh Order
    /// 0 = Top to Bottom
    /// 1 = Bottom to Top
    #define MADCTL_ML       0x10
    /// RGB-BGR Order
    /// 0 = RGB
    /// 1 = BGR
    #define MADCTL_BGR      0x08
    /// Horizontal Refresh Order
    /// 0 = Left to Right
    /// 1 = Right to Left
    #define MADCTL_MH       0x04

    #define MADCTL_PORTRAIT MADCTL_MX | MADCTL_MV
    #define MADCTL_LANDSCAPE 0
    #define MADCTL_PORTRAIT_FLIP MADCTL_MY | MADCTL_MV
    #define MADCTL_LANDSCAPE_FLIP MADCTL_MY | MADCTL_MX
// Vertical Scrolling Start Address
#define DISP_CMD_VSCRSADD   0x37
// Idle Mode OFF
#define DISP_CMD_IDMOFF     0x38
// Idle Mode ON
#define DISP_CMD_IDMON      0x39
// Pixel Format Set
#define DISP_CMD_COLMOD     0x3A
    /// 16-bit/pixel Color Format
    #define COLMOD_16BIT    0x55
    /// 18-bit/pixel Color Format
    #define COLMOD_18BIT    0x66


DRAM_ATTR static const _display_init_cmd_t _display_init_ILI9342C[] = {
#if DISP_PIN_RST <= 0
    // Software reset if hardware reset wasn't possible (RST pin not connected to board)
    {DISP_CMD_SWRESET, {0}, 3<<5},   // 150 ms delay
#endif
    {DISP_CMD_MADCTL, {MADCTL_BGR | MADCTL_LANDSCAPE}, 1},
#if DISP_DEPTH == 16
    {DISP_CMD_COLMOD, {COLMOD_16BIT}, 1},
#elif DISP_DEPTH == 18
    {DISP_CMD_COLMOD, {COLMOD_18BIT}, 1},
#endif
    {DISP_CMD_SLPOUT, {0}, 2<<5},   // Sleep out; 100 ms delay
    {DISP_CMD_DISPON, {0}, 2<<5},   // Display on; 100 ms delay
    {DISP_CMD_INVON, {0}, 0},       // Invert display; no delay
    {DISP_CMD_NOP, {0}, 0xFF}
};