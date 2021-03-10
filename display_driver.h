#pragma once

#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "display_conf.h"


typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t datalen; // Data length; bits[7:5] = delay/50; 0xFF = End of CMDs
    // Delay after command when the 3 first bits differ from 000 and 111
    // Delay = ((datalen >> 5) & 0x7) * 50 ms
    // Since bits are from 001 to 110, only these delays are allowed:
    // [50, 100, 150, 200, 250, 300]
} _display_init_cmd_t;


typedef uint16_t color16_t;
typedef struct __attribute__((__packed__)) color24_t {
    uint8_t r;
	uint8_t g;
	uint8_t b;
} color24_t;

/* Initialize SPI communication and display according to "display_conf.h"
 * settings.
 */
esp_err_t display_init();

/* Send a 24-bit color buffer to the display on specific coordinates.
 * 
 * IMPORTANT: The color buffer should be DMA-capable memory. Otherwise, the
 * efficiency of the transaction will be affected because the driver would
 * make a copy of it.
 */
esp_err_t display_send_color24(
    uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2,
    color24_t* color, size_t len);

/* Send a 16-bit color buffer to the display on specific coordinates.
 * 
 * IMPORTANT: The color buffer should be DMA-capable memory. Otherwise, the
 * efficiency of the transaction will be affected because the driver would
 * make a copy of it.
 */
esp_err_t display_send_color16(
    uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2,
    color16_t* color, size_t len);

esp_err_t display_sync();

/* Set backlight intensity level.
 * 
 *  @param value Backlight intensity as a percentage [0-100]
 */
esp_err_t display_set_backlight(uint8_t value);