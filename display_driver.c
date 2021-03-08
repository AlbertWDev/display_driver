
#include "display_driver.h"

static spi_device_handle_t _display_handle;
uint32_t _display_pending_transactions = 0;


#if (DISPLAY_DEVICE_TYPE == DISPLAY_DEVICE_ILI9341)
    #include "devices/ili9341.h"
    const _display_init_cmd_t* _display_init_sequence = _display_init_ILI9341;
#elif (DISPLAY_DEVICE_TYPE == DISPLAY_DEVICE_ST7735)
    #include "devices/st7735.h"
    const _display_init_cmd_t* _display_init_sequence = _display_init_ST7735;
#endif


/* Function called in IRQ context before a transmission starts.
 * Used to set the D/C line to the value indicated in the user field.
 */
void IRAM_ATTR _display_spi_pre_transfer_cb(spi_transaction_t *t) {    
    if((uint32_t)t->user)
        GPIO.out_w1ts = ((uint32_t)1 << DISP_PIN_DC);
    else
        GPIO.out_w1tc = ((uint32_t)1 << DISP_PIN_DC);
    
    // TODO: use GPIO.out1_w1tX.val/data for pins [32-34]
}

/* Initialize non-SPI pins
 */
esp_err_t _display_pins_init() {
    #if DISP_PIN_BCKL
    gpio_set_direction(DISP_PIN_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(DISP_PIN_BCKL, DISP_BCKL_DISABLE);
    #endif

    gpio_set_direction(DISP_PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_level(DISP_PIN_DC, 0);

    #if DISP_PIN_RST
    gpio_set_direction(DISP_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(DISP_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(DISP_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif

    return ESP_OK;
}


/* Send command to the display controller.
 * A polling transaction is made, so this is a blocking call that waits until the
 * transfer is complete.
 * 
 * IMPORTANT: Started polling and/or interrupt transactions must be finished before
 * calling this (polling transactions don't allow other transactions being in progress)
 * 
 * The overhead of an interrupt transaction is more than waiting for the transaction
 * to complete due to the (usually) small size of command transactions. For bigger
 * data blocks, _display_send_data is preferred.
 */
esp_err_t _display_write_cmd(spi_device_handle_t display_handle,
                                const uint8_t cmd,
                                const uint8_t *params, size_t len) {
    esp_err_t ret;
    static spi_transaction_t t[2];
    memset(t, 0, 2*sizeof(spi_transaction_t));

    t[0].length = 8;
    t[0].user = (void*)0;
    t[0].flags = SPI_TRANS_USE_TXDATA;
    t[0].tx_data[0] = cmd;
    if((ret = spi_device_polling_start(display_handle, &t[0], portMAX_DELAY)) != ESP_OK) return ret;

    if(len == 0 || params == NULL)
        return spi_device_polling_end(display_handle, portMAX_DELAY);
    
    t[1].length = len * 8;
    t[1].user = (void*)1;
    t[1].tx_buffer = params;
    if((ret = spi_device_polling_end(display_handle, portMAX_DELAY)) != ESP_OK) return ret;
    return spi_device_polling_transmit(display_handle, &t[1]);
}

esp_err_t _display_set_transfer_addrwin(
    spi_device_handle_t display_handle,
    uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2)
{
    esp_err_t ret;
    static spi_transaction_t t[2];
    memset(t, 0, 2*sizeof(spi_transaction_t));

    // Send column range command (8 bits)
    t[0].length = 8;
    t[0].user = (void*)0;
    t[0].flags = SPI_TRANS_USE_TXDATA;
    t[0].tx_data[0] = DISP_CMD_CASET;    // Column Address Set
    if((ret = spi_device_polling_start(display_handle, &t[0], portMAX_DELAY)) != ESP_OK) return ret;

    // Send column range data (4*8 bits)
    t[1].length = 32;
    t[1].user = (void*)1;
    t[1].flags = SPI_TRANS_USE_TXDATA;
    x1 += DISP_X_OFFSET;
    x2 += DISP_X_OFFSET;
    t[1].tx_data[0] = x1 >> 8;              // Column Start (High)
    t[1].tx_data[1] = x1 & 0xFF;            // Column Start (Low)
    t[1].tx_data[2] = x2 >> 8;              // Column End (High)
    t[1].tx_data[3] = x2 & 0xFF;            // Column End (Low)
    if((ret = spi_device_polling_end(display_handle, portMAX_DELAY)) != ESP_OK) return ret;
    if((ret = spi_device_polling_start(display_handle, &t[1], portMAX_DELAY)) != ESP_OK) return ret;

    // Send row range command (8 bits)
    t[0].tx_data[0] = DISP_CMD_RASET;    // Row Address Set
    if((ret = spi_device_polling_end(display_handle, portMAX_DELAY)) != ESP_OK) return ret;
    if((ret = spi_device_polling_start(display_handle, &t[0], portMAX_DELAY)) != ESP_OK) return ret;

    // Send row range data (4*8 bits)
    y1 += DISP_Y_OFFSET;
    y2 += DISP_Y_OFFSET;
    t[1].tx_data[0] = y1 >> 8;              // Page Start (High)
    t[1].tx_data[1] = y1 & 0xFF;            // Page Start (Low)
    t[1].tx_data[2] = y2 >> 8;              // Page End (High)
    t[1].tx_data[3] = y2 & 0xFF;            // Page End (Low)
    if((ret = spi_device_polling_end(display_handle, portMAX_DELAY)) != ESP_OK) return ret;
    return spi_device_polling_transmit(display_handle, &t[1]);
}

esp_err_t _display_send_data(
    spi_device_handle_t display_handle,
    const uint8_t* data, uint32_t len)
{
    esp_err_t ret;
    static spi_transaction_t t[2];
    memset(t, 0, 2*sizeof(spi_transaction_t));

    // CMD: Prepare display for data transfer
    t[0].length = 8;
    t[0].user = (void*)0;
    t[0].flags = SPI_TRANS_USE_TXDATA;
    t[0].tx_data[0] = DISP_CMD_RAMWR;    // Memory Write
    if((ret = spi_device_queue_trans(display_handle, &t[0], portMAX_DELAY)) != ESP_OK) return ret;
    _display_pending_transactions++;

    // DATA: Transfer color data
    t[1].user = (void*)1;
    t[1].length = len * 8;
    t[1].tx_buffer = data;
    if((ret = spi_device_queue_trans(display_handle, &t[1], portMAX_DELAY)) != ESP_OK) return ret;
    _display_pending_transactions++;

    return ESP_OK;
}

esp_err_t _display_send_data_finish(spi_device_handle_t display_handle) {
    esp_err_t ret;
    spi_transaction_t* t;

    while(_display_pending_transactions > 0) {
        if((ret = spi_device_get_trans_result(display_handle, &t, portMAX_DELAY)) != ESP_OK) return ret;
        _display_pending_transactions--;
    }
    return ESP_OK;
}

esp_err_t _display_init(spi_device_handle_t display_handle) {
    esp_err_t ret;

    // Wait in case previous transactions are still in progress
    // The polling transactions can NOT be started if there are any other transactions in
    // progress
    if((ret = _display_send_data_finish(_display_handle)) != ESP_OK) return ret;

    uint8_t delay_ms = 0;
    for(int cmd = 0; _display_init_sequence[cmd].datalen != 0xFF; cmd++) {
        ret = _display_write_cmd(display_handle,
            _display_init_sequence[cmd].cmd,
            _display_init_sequence[cmd].data,
            _display_init_sequence[cmd].datalen & 0x1F);
        if(ret != ESP_OK) return ret;
        
        if((delay_ms = ((_display_init_sequence[cmd].datalen >> 5) & 0x7) * 50))
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    return ESP_OK;
}

esp_err_t display_init() {
    esp_err_t ret = ESP_OK;

    /// Initialize pins
    _display_pins_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    /// Configure SPI bus
    spi_bus_config_t disp_spi_bus_cfg = {
        .miso_io_num = DISP_PIN_MISO,
        .mosi_io_num = DISP_PIN_MOSI,
        .sclk_io_num = DISP_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_SPI_MAX_TRANSFER_SIZE
    };
    ret = spi_bus_initialize(DISP_SPI_HOST, &disp_spi_bus_cfg, 1);

    /// Attach display device
    spi_device_interface_config_t disp_spi_dev_cfg = {
        .clock_speed_hz = DISP_SPI_CLOCK_SPEED,
        .mode = 0,
        .spics_io_num = DISP_PIN_CS,
        .queue_size = DISP_TRANSACTION_QUEUE_SIZE,
        .pre_cb =_display_spi_pre_transfer_cb   // Pre-transfer callback to handle D/C line
    };
    ret = spi_bus_add_device(DISP_SPI_HOST, &disp_spi_dev_cfg, &_display_handle);

    /// Initialize display with commands sequence
    ret = _display_init(_display_handle);

    #if DISP_PIN_BCKL
    gpio_set_level(DISP_PIN_BCKL, DISP_BCKL_ENABLE);
    #endif

    return ret;
}

esp_err_t display_send_color24(
    uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2,
    color24_t* color, size_t len)
{
    esp_err_t ret;

    // Wait in case previous transactions are still in progress
    // The polling transaction (addrwin) can NOT be started if there are any other transactions in
    // progress
    if((ret = _display_send_data_finish(_display_handle)) != ESP_OK) return ret;

    /// Set address window for transfer data
    if((ret = _display_set_transfer_addrwin(_display_handle, x1, y1, x2, y2)) != ESP_OK) return ret;

    /// Transfer color data to display
    if((ret = _display_send_data(_display_handle, (const uint8_t*)color, len * 3)) != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t display_send_color16(
    uint16_t x1, uint16_t y1,
    uint16_t x2, uint16_t y2,
    color16_t* color, size_t len)
{
    esp_err_t ret;

    // Wait in case previous transactions are still in progress
    // The polling transaction (addrwin) can NOT be started if there are any other transactions in
    // progress
    if((ret = _display_send_data_finish(_display_handle)) != ESP_OK) return ret;

    /// Set address window for transfer data
    if((ret = _display_set_transfer_addrwin(_display_handle, x1, y1, x2, y2)) != ESP_OK) return ret;
    
    /// Transfer color data to display
    if((ret = _display_send_data(_display_handle, (const uint8_t*)color, len * 2)) != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t display_sync() {
    return _display_send_data_finish(_display_handle);
}