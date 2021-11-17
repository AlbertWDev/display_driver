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
// Read Display Power
#define DISP_CMD_RDDPM      0x0A
// Read Display MADCTL
#define DISP_CMD_RDDMADCTL  0x0B
// Read Display Pixel
#define DISP_CMD_RDDCOLMOD  0x0C
// Read Display Image
#define DISP_CMD_RDDIM      0x0D
// Read Display Signal
#define DISP_CMD_RDDSM      0x0E
// Sleep in & booster off
#define DISP_CMD_SLPIN      0x10
// Sleep out & booster on
#define DISP_CMD_SLPOUT     0x11
// Partial mode on
#define DISP_CMD_PTLON      0x12
// Partial off (Normal)
#define DISP_CMD_NORON      0x13
// Display inversion off
#define DISP_CMD_INVOFF     0x20
// Display inversion on
#define DISP_CMD_INVON      0x21
// Gamma curve select
#define DISP_CMD_GAMSET     0x26
    /// Gamma Curve 1
    #define GAMMA_CURVE1    0x01
    /// Gamma Curve 2
    #define GAMMA_CURVE2    0x02
    /// Gamma Curve 3
    #define GAMMA_CURVE3    0x04
    /// Gamma Curve 4
    #define GAMMA_CURVE4    0x08
// Display off
#define DISP_CMD_DISPOFF    0x28
// Display on
#define DISP_CMD_DISPON     0x29
// Column address set
#define DISP_CMD_CASET      0x2A
// Row address set
#define DISP_CMD_RASET      0x2B
// Memory write
#define DISP_CMD_RAMWR      0x2C
// Memory read
#define DISP_CMD_RAMRD      0x2E
// Partial start/end address set
#define DISP_CMD_PTLAR      0x30
// Tearing effect line off
#define DISP_CMD_TEOFF      0x34
// Tearing effect mode set & on
#define DISP_CMD_TEON       0x35
// Memory data access control
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
// Idle mode off
#define DISP_CMD_IDMOFF     0x38
// Idle mode on
#define DISP_CMD_IDMON      0x39
// Interface pixel format
#define DISP_CMD_COLMOD     0x3A
    /// 65K RGB Interface Color Format
    #define COLMOD_65K      0x50
    /// 262K RGB Interface Color Format
    #define COLMOD_262K     0x60
    /// 12-bit/pixel Control Interface Color Format
    #define COLMOD_12BIT    0x03
    /// 16-bit/pixel Control Interface Color Format
    #define COLMOD_16BIT    0x05
    /// 18-bit/pixel Control Interface Color Format
    #define COLMOD_18BIT    0x06
// Porch Setting
#define DISP_CMD_PORCTRL    0xB2
// Gate Control
#define DISP_CMD_GCTRL      0xB7
// VCOM Setting
#define DISP_CMD_VCOMS      0xBB
// LCM Control
#define DISP_CMD_LCMCTRL    0xC0
// VDV and VRH Command Enable
#define DISP_CMD_VDVVRHEN   0xC2
// VRH Set
#define DISP_CMD_VRHS       0xC3
// VDV Set
#define DISP_CMD_VDVS       0xC4
// Frame Rate Control in Normal Mode
#define DISP_CMD_FRCTRL2    0xC6
// Power Control 1
#define DISP_CMD_PWCTRL1    0xD0


DRAM_ATTR static const _display_init_cmd_t _display_init_ST7789VW[] = {
#if DISP_PIN_RST <= 0
    // Software reset if hardware reset wasn't possible (RST pin not connected to board)
    {DISP_CMD_SWRESET, {0}, 3<<5},  // 150ms delay (120ms required)
#endif
    //{DISP_CMD_MADCTL, {MADCTL_MX|MADCTL_MY}, 1},
    {DISP_CMD_MADCTL, {MADCTL_MY|MADCTL_MX}, 1},
#if DISP_DEPTH == 16
    {DISP_CMD_COLMOD, {COLMOD_16BIT}, 1},
#elif DISP_DEPTH == 18
    {DISP_CMD_COLMOD, {COLMOD_18BIT}, 1},
#endif
    //{DISP_CMD_PORCTRL, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5},
    //{DISP_CMD_GCTRL, {0x35}, 1},
    //{DISP_CMD_VCOMS, {0x1F}, 1},
    //{DISP_CMD_LCMCTRL, {0x2C}, 1},
	//{DISP_CMD_VDVVRHEN, {0x01}, 1},
	//{DISP_CMD_VRHS, {0x12}, 1},
	//{DISP_CMD_VDVS, {0x20}, 1},
	//{DISP_CMD_FRCTRL2, {0x0F}, 1},
	//{DISP_CMD_PWCTRL1, {0xA4, 0xA1}, 2},
    {DISP_CMD_SLPOUT, {0}, 3<<5},   // 150ms delay (120ms required)
    {DISP_CMD_DISPON, {0}, 3<<5},   // 150ms delay (120ms required before DISPOFF)
    {DISP_CMD_NOP, {0}, 0xFF}
};
