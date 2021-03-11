#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "display_driver.h"

#include "esp_log.h"
static const char* TAG = "EXAMPLE-FPS";

#define BUFFER_SIZE DISP_WIDTH * DISP_HEIGHT

#if DISP_DEPTH == 16
    typedef color16_t color_t;
    #define display_send_color display_send_color16
#elif DISP_DEPTH == 18
    typedef color24_t color_t;
    #define display_send_color display_send_color24
#endif

static int64_t elapsed;

void fps_task(void *pvParameter)
{
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "FPS: %f (%lld us/frame)", 1000000./elapsed, elapsed);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    if(display_init() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize display\n");
        return;
    }

    color_t* buf = heap_caps_malloc(BUFFER_SIZE * sizeof(color_t), MALLOC_CAP_DMA);
    xTaskCreate(&fps_task, "fps", 2048, NULL, 5, NULL);

    uint8_t i = 0;
    while(1) {
        uint64_t start = esp_timer_get_time();

        // Fill buffer with black or white alternatively
        memset(buf, (i++ % 2) ? 0xFF : 0x00, BUFFER_SIZE * sizeof(color_t));

        // Push buffer to display
        if(display_send_color(0, 0, DISP_WIDTH-1, DISP_HEIGHT-1, buf, BUFFER_SIZE) != ESP_OK) {
            ESP_LOGE(TAG, "Unable to send data to display\n");
            return;
        }

        elapsed = esp_timer_get_time() - start;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}