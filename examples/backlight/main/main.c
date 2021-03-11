#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "display_driver.h"

#include "esp_log.h"
static const char* TAG = "EXAMPLE-BCKL";

#define BCKL_COUNT 6
static const uint8_t bckl_levels[BCKL_COUNT] = {0, 20, 40, 60, 80, 100};


#define BUFFER_SIZE DISP_WIDTH * DISP_HEIGHT

#if DISP_DEPTH == 16
    typedef color16_t color_t;
    static color_t WHITE = 0xFFFF;
    static color_t GRAY = 0x8888;
    #define display_send_color display_send_color16
#elif DISP_DEPTH == 18
    typedef color24_t color_t;
    static color_t WHITE = {.r = 0xFF, .g = 0xFF, .b = 0xFF};
    static color_t GRAY = {.r = 0x7F, .g = 0x7F, .b = 0x7F};
    #define display_send_color display_send_color24
#endif

/*
 * Draw a rectangle of the given color in the buffer
 */
void draw_rect(color_t* buf, color_t color, int16_t x, int16_t y, int16_t w, int16_t h) {
    // Move to first row affected by the rectangle
    buf = &buf[y * DISP_WIDTH];

    // Fill first row with color
    for(int16_t _x = x; _x < x+w; _x++)
        memcpy(&buf[_x], &color, sizeof(color_t));

    // Store first row reference
    color_t* first_row = &buf[x];
    size_t first_row_width = w * sizeof(color_t);   // Width in bytes of the first row in the rectangle
    buf += DISP_WIDTH; // Skip first row

    // Copy first row to the rest of rows
    for(int16_t row = y + 1; row < y + h; row++) {
        memcpy(&buf[x], first_row, first_row_width);
        buf += DISP_WIDTH; // Next row
    }
}

/* 
 * Draw the backlight level indicator
 */
void draw_backlight_level(color_t* buf, uint8_t level) {
    static const uint8_t padding = 4;
    uint8_t indicator_width = DISP_WIDTH - (BCKL_COUNT + 1) * padding;

    color_t color;
    for(uint8_t i = 0; i < BCKL_COUNT; i++) {
        color = (bckl_levels[i] <= level) ? WHITE : GRAY;
        draw_rect(buf, color,
            padding + i * (indicator_width + padding), 5,
            indicator_width, 20);
    }
}

void app_main()
{
    uint8_t bckl_index = 0;
    int8_t change_direction = 1;

    if(display_init() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize display\n");
        return;
    }

    color_t* buf = heap_caps_malloc(BUFFER_SIZE * sizeof(color_t), MALLOC_CAP_DMA);

    while(1) {
        // Clear display buffer
        memset(buf, 0, BUFFER_SIZE * sizeof(color_t));

        // Update backlight level
        uint8_t level = bckl_levels[bckl_index];
        display_set_backlight(level);
        draw_backlight_level(buf, level);

        // Push buffer to display
        if(display_send_color(0, 0, DISP_WIDTH-1, DISP_HEIGHT-1, buf, BUFFER_SIZE) != ESP_OK) {
            ESP_LOGE(TAG, "Unable to send data to display\n");
            return;
        }        

        if(bckl_index == 0)
            change_direction = 1;
        else if(bckl_index == BCKL_COUNT - 1)
            change_direction = -1;
        bckl_index += change_direction;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}