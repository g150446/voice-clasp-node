#include "display.h"

#include <stdio.h>
#include <string.h>

#include "axp192.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"

static const char *TAG = "hn_disp";

#define LCD_HOST           SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_CMD_BITS       8
#define LCD_PARAM_BITS     8

#define PIN_LCD_SCLK 13
#define PIN_LCD_MOSI 15
#define PIN_LCD_DC   23
#define PIN_LCD_RST  18
#define PIN_LCD_CS   5

#define LCD_H_RES 135
#define LCD_V_RES 240
#define LCD_GAP_X 52
#define LCD_GAP_Y 40

/*
 * These are what the panel ACTUALLY shows, measured with the `p` test pattern
 * (four bands of 0xF800 / 0x07E0 / 0x001F / 0xFFFF) on real hardware. The
 * channel order does not match a plain RGB565 reading of the values, so do not
 * "correct" them from the numbers alone - re-measure with the test pattern.
 */
#define COLOR_BG     0x0000
#define COLOR_WHITE  0xFFFF
#define COLOR_BLUE   0x07E0
#define COLOR_RED    0xF800

/* 5x7 glyphs, bit4 = leftmost pixel. Index 0-9, 10='%', 11='+'. */
static const uint8_t font5x7[][7] = {
    { 0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e },
    { 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e },
    { 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f },
    { 0x0e, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0e },
    { 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 },
    { 0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e },
    { 0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e },
    { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
    { 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e },
    { 0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c },
    { 0x19, 0x1a, 0x04, 0x08, 0x16, 0x13, 0x00 },
    { 0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00 },
    { 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 },
    { 0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e },
    { 0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e },
    { 0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e },
    { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f },
    { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
    { 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00 },
    { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f },
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04 },
    { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f },
    { 0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e },
    { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 },
    { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
};
#define FONT_W 5
#define FONT_H 7
#define FONT_SCALE 4
#define FONT_PAD 2
#define BAT_ICON_W 36
#define BAT_ICON_H 18
#define BAT_NUB_W 4
#define BAT_NUB_H 10

#define BMP_NOT_CONNECTED_W 105
#define BMP_NOT_CONNECTED_H 39
static const uint8_t bmp_not_connected[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1D, 0xE0, 0x3C, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1F, 0xF0, 0xFF, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1E, 0x38, 0xC3, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x39, 0xC3, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x19, 0x81, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x19, 0x81, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x19, 0x81, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x19, 0x81, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x19, 0xC3, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x18, 0xC3, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x38, 0xFF, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x38, 0x3C, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x1C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x18, 0x00,
  0x03, 0xC0, 0x78, 0x3B, 0xC3, 0xBC, 0x07, 0x80, 0xF1, 0xFC, 0x1E, 0x07, 0xD8, 0x00,
  0x0F, 0xE1, 0xFE, 0x3F, 0xE3, 0xFE, 0x1F, 0xC3, 0xF9, 0xFC, 0x7F, 0x0F, 0xF8, 0x00,
  0x0C, 0x71, 0x86, 0x3C, 0x73, 0xC7, 0x18, 0xE3, 0x1C, 0x70, 0x63, 0x8C, 0x38, 0x00,
  0x1C, 0x33, 0x87, 0x38, 0x73, 0x87, 0x30, 0x67, 0x0C, 0x70, 0xC1, 0x9C, 0x38, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x30, 0x76, 0x00, 0x70, 0xC1, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x3F, 0xF6, 0x00, 0x70, 0xFF, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x3F, 0xF6, 0x00, 0x70, 0xFF, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x30, 0x06, 0x00, 0x70, 0xC0, 0x18, 0x18, 0x00,
  0x1C, 0x33, 0x87, 0x38, 0x33, 0x83, 0x30, 0x67, 0x0C, 0x70, 0xC1, 0x98, 0x38, 0x00,
  0x0C, 0x71, 0x86, 0x38, 0x33, 0x83, 0x18, 0x63, 0x1C, 0x70, 0x61, 0x9C, 0x38, 0x00,
  0x0F, 0xE1, 0xFE, 0x38, 0x73, 0x87, 0x1F, 0xE3, 0xF8, 0x3C, 0x7F, 0x8F, 0xF8, 0x00,
  0x03, 0xC0, 0x78, 0x38, 0x73, 0x87, 0x07, 0x80, 0xF0, 0x3C, 0x1E, 0x07, 0xDC, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
#define BMP_CONNECTED_W 105
#define BMP_CONNECTED_H 20
static const uint8_t bmp_connected[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x1C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x18, 0x00,
  0x03, 0xC0, 0x78, 0x3B, 0xC3, 0xBC, 0x07, 0x80, 0xF1, 0xFC, 0x1E, 0x07, 0xD8, 0x00,
  0x0F, 0xE1, 0xFE, 0x3F, 0xE3, 0xFE, 0x1F, 0xC3, 0xF9, 0xFC, 0x7F, 0x0F, 0xF8, 0x00,
  0x0C, 0x71, 0x86, 0x3C, 0x73, 0xC7, 0x18, 0xE3, 0x1C, 0x70, 0x63, 0x8C, 0x38, 0x00,
  0x1C, 0x33, 0x87, 0x38, 0x73, 0x87, 0x30, 0x67, 0x0C, 0x70, 0xC1, 0x9C, 0x38, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x30, 0x76, 0x00, 0x70, 0xC1, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x3F, 0xF6, 0x00, 0x70, 0xFF, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x3F, 0xF6, 0x00, 0x70, 0xFF, 0xD8, 0x18, 0x00,
  0x18, 0x03, 0x03, 0x38, 0x33, 0x83, 0x30, 0x06, 0x00, 0x70, 0xC0, 0x18, 0x18, 0x00,
  0x1C, 0x33, 0x87, 0x38, 0x33, 0x83, 0x30, 0x67, 0x0C, 0x70, 0xC1, 0x98, 0x38, 0x00,
  0x0C, 0x71, 0x86, 0x38, 0x33, 0x83, 0x18, 0x63, 0x1C, 0x70, 0x61, 0x9C, 0x38, 0x00,
  0x0F, 0xE1, 0xFE, 0x38, 0x73, 0x87, 0x1F, 0xE3, 0xF8, 0x3C, 0x7F, 0x8F, 0xF8, 0x00,
  0x03, 0xC0, 0x78, 0x38, 0x73, 0x87, 0x07, 0x80, 0xF0, 0x3C, 0x1E, 0x07, 0xDC, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
#define BMP_RECORDING_W 98
#define BMP_RECORDING_H 23
static const uint8_t bmp_recording[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x1D, 0xE1, 0xE0, 0x3C, 0x07, 0x83, 0xBC, 0x7D, 0x8E, 0x77, 0x81, 0xF6, 0x00,
  0x1F, 0xC7, 0xF0, 0xFE, 0x1F, 0xE3, 0xF8, 0xFF, 0x8E, 0x7F, 0xC3, 0xFE, 0x00,
  0x1E, 0x06, 0x38, 0xC7, 0x18, 0x63, 0xC0, 0xC3, 0x8E, 0x78, 0xE7, 0x0E, 0x00,
  0x1C, 0x0C, 0x19, 0xC3, 0x38, 0x73, 0x81, 0xC3, 0x8E, 0x70, 0xE6, 0x0E, 0x00,
  0x1C, 0x0C, 0x1D, 0x80, 0x30, 0x33, 0x81, 0x81, 0x8E, 0x70, 0x66, 0x06, 0x00,
  0x1C, 0x0F, 0xFD, 0x80, 0x30, 0x33, 0x81, 0x81, 0x8E, 0x70, 0x66, 0x06, 0x00,
  0x1C, 0x0F, 0xFD, 0x80, 0x30, 0x33, 0x81, 0x81, 0x8E, 0x70, 0x66, 0x06, 0x00,
  0x1C, 0x0C, 0x01, 0x80, 0x30, 0x33, 0x81, 0x81, 0x8E, 0x70, 0x67, 0x0E, 0x00,
  0x1C, 0x0C, 0x19, 0xC3, 0x38, 0x73, 0x81, 0x83, 0x8E, 0x70, 0x67, 0x1E, 0x00,
  0x1C, 0x06, 0x18, 0xC7, 0x18, 0x63, 0x81, 0xC3, 0x8E, 0x70, 0x63, 0xFE, 0x00,
  0x1C, 0x07, 0xF8, 0xFE, 0x1F, 0xE3, 0x80, 0xFF, 0x8E, 0x70, 0xE0, 0xC6, 0x00,
  0x1C, 0x01, 0xE0, 0x3C, 0x07, 0x83, 0x80, 0x7D, 0xCE, 0x70, 0xE0, 0x0E, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFC, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static esp_lcd_panel_handle_t s_panel;
static bool s_ready;
static display_status_t s_status = DISPLAY_STATUS_NOT_CONNECTED;
static bool s_parked;
static uint8_t s_bat_pct = 255;
static bool s_bat_chg;
static uint8_t s_drawn_bat = 255;
static bool s_drawn_chg;
static bool s_peer_on;
static bool s_peer_ota;
static uint8_t s_peer_mac[6];
static bool s_drawn_peer_on;
static bool s_drawn_peer_ota;
static uint8_t s_drawn_peer_mac[6];

/*
 * The panel is driven from one task only. display_set_status() is called from
 * button_task, audio_stream_task, uart_task, the NimBLE host task and
 * app_main; a full repaint is ~240 SPI transactions, so letting those callers
 * paint directly both raced on the SPI device (leaving the screen black and
 * the caller stuck) and stalled the audio pipeline mid-recording.
 */
static SemaphoreHandle_t s_lock;              /* guards all panel access */
static QueueHandle_t s_req;                   /* depth 1, overwrite: wanted status */
static TaskHandle_t s_task;
static int s_drawn = -1;                      /* last painted status, -1 = unknown */
static uint16_t *s_line;

#define DISPLAY_LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define DISPLAY_UNLOCK() xSemaphoreGive(s_lock)

static void display_task(void *arg);
static void draw_battery(void);

static void bl_set(bool on)
{
    axp192_set_ldo2(on);
}

static esp_err_t fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_ready || s_line == NULL || w <= 0 || h <= 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > LCD_H_RES) {
        w = LCD_H_RES - x;
    }
    if (y + h > LCD_V_RES) {
        h = LCD_V_RES - y;
    }
    if (w <= 0 || h <= 0) {
        return ESP_OK;
    }
    for (int i = 0; i < w; i++) {
        s_line[i] = color;
    }
    esp_err_t ret = ESP_OK;
    for (int row = 0; row < h; row++) {
        ret = esp_lcd_panel_draw_bitmap(s_panel, x, y + row, x + w, y + row + 1, s_line);
        if (ret != ESP_OK) {
            break;
        }
    }
    return ret;
}

static esp_err_t blit_mono_bitmap(int x0, int y0, int bw, int bh,
                                  const uint8_t *bmp, uint16_t fg, uint16_t bg)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_line == NULL || bw <= 0 || bh <= 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x0 < 0 || y0 < 0 || x0 + bw > LCD_H_RES || y0 + bh > LCD_V_RES) {
        return ESP_ERR_INVALID_ARG;
    }
    const int row_bytes = (bw + 7) / 8;
    esp_err_t ret = ESP_OK;
    for (int y = 0; y < bh; y++) {
        const uint8_t *src = bmp + y * row_bytes;
        for (int x = 0; x < bw; x++) {
            const uint8_t bit = src[x / 8] & (0x80u >> (x & 7));
            s_line[x] = bit ? fg : bg;
        }
        ret = esp_lcd_panel_draw_bitmap(s_panel, x0, y0 + y, x0 + bw, y0 + y + 1, s_line);
        if (ret != ESP_OK) {
            break;
        }
    }
    return ret;
}

esp_err_t display_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_line == NULL) {
        s_line = heap_caps_malloc(LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
        if (s_line == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    bl_set(false);

    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 40 * sizeof(uint16_t),
    };
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_io_spi: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        /* Colour constants above were measured with this setting - changing it
         * invalidates them. */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(io, &panel_config, &s_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "st7789: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    s_ready = true;

    if (s_task == NULL) {
        s_req = xQueueCreate(1, sizeof(display_status_t));
        if (s_req == NULL) {
            return ESP_ERR_NO_MEM;
        }
        if (xTaskCreate(display_task, "display", 4096, NULL, 3, &s_task) != pdPASS) {
            vQueueDelete(s_req);
            s_req = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "LCD ready %dx%d gap(%d,%d)", LCD_H_RES, LCD_V_RES, LCD_GAP_X, LCD_GAP_Y);
    return ESP_OK;
}

static int glyph_index(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c == '%') {
        return 10;
    }
    if (c == '+') {
        return 11;
    }
    if (c >= 'A' && c <= 'F') {
        return 12 + (c - 'A');
    }
    if (c >= 'a' && c <= 'f') {
        return 12 + (c - 'a');
    }
    if (c == ':') {
        return 18;
    }
    if (c == 'T' || c == 't') {
        return 19;
    }
    if (c == 'V' || c == 'v') {
        return 20;
    }
    if (c == 'L' || c == 'l') {
        return 21;
    }
    if (c == 'I' || c == 'i') {
        return 22;
    }
    if (c == 'N' || c == 'n') {
        return 23;
    }
    if (c == 'K' || c == 'k') {
        return 24;
    }
    if (c == 'O' || c == 'o') {
        return 0;
    }
    return -1;
}

static void blit_glyph_scaled(int x, int y, int gi, uint16_t fg, int scale)
{
    if (gi < 0 || s_line == NULL || scale < 1) {
        return;
    }
    const int dw = FONT_W * scale;
    const int dh = FONT_H * scale;
    if (x < 0 || y < 0 || x + dw > LCD_H_RES || y + dh > LCD_V_RES) {
        return;
    }
    for (int row = 0; row < FONT_H; row++) {
        uint8_t bits = font5x7[gi][row];
        int px = 0;
        for (int col = 0; col < FONT_W; col++) {
            uint16_t c = (bits & (0x10 >> col)) ? fg : COLOR_BG;
            for (int s = 0; s < scale; s++) {
                s_line[px++] = c;
            }
        }
        for (int s = 0; s < scale; s++) {
            (void)esp_lcd_panel_draw_bitmap(s_panel, x, y + row * scale + s,
                                            x + dw, y + row * scale + s + 1, s_line);
        }
    }
}

static void blit_glyph(int x, int y, int gi, uint16_t fg)
{
    blit_glyph_scaled(x, y, gi, fg, FONT_SCALE);
}

static void draw_text_scaled(int x, int y, const char *s, uint16_t fg, int scale, int pad)
{
    const int cell = FONT_W * scale + pad;
    for (const char *p = s; *p; p++) {
        int gi = glyph_index(*p);
        if (gi >= 0) {
            blit_glyph_scaled(x, y, gi, fg, scale);
            x += cell;
        } else if (*p == ' ') {
            x += cell;
        }
    }
}

static void draw_battery(void)
{
    if (!s_ready || s_bat_pct > 100) {
        return;
    }

    char buf[8];
    snprintf(buf, sizeof(buf), "%u%%", (unsigned)s_bat_pct);

    const int cell = FONT_W * FONT_SCALE + FONT_PAD;
    int text_w = 0;
    for (const char *p = buf; *p; p++) {
        if (glyph_index(*p) >= 0) {
            text_w += cell;
        }
    }
    const int icon_w = BAT_ICON_W + BAT_NUB_W;
    const int text_h = FONT_H * FONT_SCALE;
    const int gap = 6;
    const int total_w = icon_w + gap + text_w;
    int x = (LCD_H_RES - total_w) / 2;
    if (x < 0) {
        x = 0;
    }
    const int y = 8;
    const int iy = y + (text_h - BAT_ICON_H) / 2;
    uint16_t fg = COLOR_WHITE;
    if (s_bat_chg) {
        fg = COLOR_BLUE;
    } else if (s_bat_pct <= 15) {
        fg = COLOR_RED;
    }

    (void)fill_rect(x, iy, BAT_ICON_W, BAT_ICON_H, fg);
    (void)fill_rect(x + 2, iy + 2, BAT_ICON_W - 4, BAT_ICON_H - 4, COLOR_BG);
    (void)fill_rect(x + BAT_ICON_W, iy + (BAT_ICON_H - BAT_NUB_H) / 2, BAT_NUB_W, BAT_NUB_H, fg);

    int inner_w = BAT_ICON_W - 8;
    int fill_w = (inner_w * (int)s_bat_pct) / 100;
    if (fill_w > 0) {
        (void)fill_rect(x + 4, iy + 4, fill_w, BAT_ICON_H - 8, fg);
    }

    int tx = x + icon_w + gap;
    const int ty = y;
    for (const char *p = buf; *p; p++) {
        int gi = glyph_index(*p);
        if (gi >= 0) {
            blit_glyph(tx, ty, gi, fg);
            tx += cell;
        }
    }
}

static void draw_status(display_status_t status)
{
    const uint8_t *bmp = bmp_not_connected;
    int bw = BMP_NOT_CONNECTED_W;
    int bh = BMP_NOT_CONNECTED_H;
    uint16_t fg = COLOR_WHITE;

    switch (status) {
    case DISPLAY_STATUS_CONNECTED:
        bmp = bmp_connected;
        bw = BMP_CONNECTED_W;
        bh = BMP_CONNECTED_H;
        fg = COLOR_BLUE;
        break;
    case DISPLAY_STATUS_RECORDING:
        bmp = bmp_recording;
        bw = BMP_RECORDING_W;
        bh = BMP_RECORDING_H;
        fg = COLOR_RED;
        break;
    case DISPLAY_STATUS_ADVERTISING:
    case DISPLAY_STATUS_LINKING:
        bmp = NULL;
        break;
    case DISPLAY_STATUS_NOT_CONNECTED:
    default:
        break;
    }

    (void)fill_rect(0, 0, LCD_H_RES, LCD_V_RES, COLOR_BG);
    const int bat_band = 8 + FONT_H * FONT_SCALE + 8;
    int x = (LCD_H_RES - bw) / 2;
    int y = bat_band + (LCD_V_RES - bat_band - bh) / 2;
    if (status == DISPLAY_STATUS_ADVERTISING || status == DISPLAY_STATUS_LINKING) {
        const char *label = (status == DISPLAY_STATUS_LINKING) ? "LINK" : "ADV";
        const int nch = (status == DISPLAY_STATUS_LINKING) ? 4 : 3;
        const int cell = FONT_W * FONT_SCALE + FONT_PAD;
        const int tw = nch * cell;
        x = (LCD_H_RES - tw) / 2;
        y = bat_band + (LCD_V_RES - bat_band - FONT_H * FONT_SCALE) / 2;
        draw_text_scaled(x, y, label, COLOR_WHITE, FONT_SCALE, FONT_PAD);
        bh = FONT_H * FONT_SCALE;
    } else {
        (void)blit_mono_bitmap(x, y, bw, bh, bmp, fg, COLOR_BG);
    }
    if (s_peer_on) {
        char line[24];
        if (s_peer_ota) {
            snprintf(line, sizeof(line), "OTA %02X:%02X:%02X:%02X:%02X:%02X",
                     s_peer_mac[5], s_peer_mac[4], s_peer_mac[3],
                     s_peer_mac[2], s_peer_mac[1], s_peer_mac[0]);
        } else {
            snprintf(line, sizeof(line), "%02X:%02X:%02X:%02X:%02X:%02X",
                     s_peer_mac[5], s_peer_mac[4], s_peer_mac[3],
                     s_peer_mac[2], s_peer_mac[1], s_peer_mac[0]);
        }
        int n = 0;
        for (const char *p = line; *p; p++) {
            if (glyph_index(*p) >= 0 || *p == ' ') {
                n++;
            }
        }
        const int peer_scale = 1;
        const int peer_pad = 1;
        const int tw = n * (FONT_W * peer_scale + peer_pad);
        int px = (LCD_H_RES - tw) / 2;
        if (px < 0) {
            px = 0;
        }
        draw_text_scaled(px, y + bh + 8, line, COLOR_WHITE, peer_scale, peer_pad);
    }
    draw_battery();
    bl_set(true);
}

static void display_task(void *arg)
{
    (void)arg;
    display_status_t want;

    while (1) {
        if (xQueueReceive(s_req, &want, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        DISPLAY_LOCK();
        /* Skip while parked for deep sleep, and skip repaints that change nothing. */
        bool peer_changed = (s_peer_on != s_drawn_peer_on) ||
                            (s_peer_ota != s_drawn_peer_ota) ||
                            (s_peer_on && memcmp(s_peer_mac, s_drawn_peer_mac, 6) != 0);
        if (!s_parked &&
            ((int)want != s_drawn || s_bat_pct != s_drawn_bat || s_bat_chg != s_drawn_chg ||
             peer_changed)) {
            draw_status(want);
            s_drawn = (int)want;
            s_drawn_bat = s_bat_pct;
            s_drawn_chg = s_bat_chg;
            s_drawn_peer_on = s_peer_on;
            s_drawn_peer_ota = s_peer_ota;
            memcpy(s_drawn_peer_mac, s_peer_mac, 6);
        }
        DISPLAY_UNLOCK();
    }
}

void display_set_status(display_status_t status)
{
    if (!s_ready && display_init() != ESP_OK) {
        return;
    }
    s_status = status;

    /* Never paint on the caller's task: just hand the wanted state over. */
    if (s_req != NULL) {
        (void)xQueueOverwrite(s_req, &status);
        return;
    }

    DISPLAY_LOCK();
    draw_status(status);
    s_drawn = (int)status;
    s_drawn_bat = s_bat_pct;
    s_drawn_chg = s_bat_chg;
    s_drawn_peer_on = s_peer_on;
    s_drawn_peer_ota = s_peer_ota;
    memcpy(s_drawn_peer_mac, s_peer_mac, 6);
    DISPLAY_UNLOCK();
}

void display_set_battery(uint8_t percent, bool charging)
{
    if (percent > 100) {
        percent = 100;
    }
    s_bat_pct = percent;
    s_bat_chg = charging;
    if (s_req != NULL) {
        display_status_t st = s_status;
        (void)xQueueOverwrite(s_req, &st);
    }
}

void display_set_peer(const uint8_t mac[6], bool present, bool ota)
{
    s_peer_on = present;
    s_peer_ota = present && ota;
    if (present && mac != NULL) {
        memcpy(s_peer_mac, mac, 6);
    } else {
        memset(s_peer_mac, 0, 6);
    }
    if (s_req != NULL) {
        display_status_t st = s_status;
        (void)xQueueOverwrite(s_req, &st);
    }
}

static void sleep_locked(void)
{
    bl_set(false);
    if (s_ready && s_panel != NULL) {
        (void)esp_lcd_panel_disp_on_off(s_panel, false);
        (void)fill_rect(0, 0, LCD_H_RES, LCD_V_RES, COLOR_BG);
    }
    s_drawn = -1;
}

/* Colour calibration aid: paint four known RGB565 values as horizontal bands,
 * top to bottom. Serial 'p'. The constants above were derived from this. */
void display_test_pattern(void)
{
    static const uint16_t bands[4] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };
    DISPLAY_LOCK();
    if (s_ready) {
        const int h = LCD_V_RES / 4;
        for (int i = 0; i < 4; i++) {
            (void)fill_rect(0, i * h, LCD_H_RES, h, bands[i]);
        }
        bl_set(true);
        s_drawn = -1;
    }
    DISPLAY_UNLOCK();
}

void display_sleep(void)
{
    DISPLAY_LOCK();
    sleep_locked();
    DISPLAY_UNLOCK();
}

static void park_pad_low(int gpio_num)
{
    if (!rtc_gpio_is_valid_gpio((gpio_num_t)gpio_num)) {
        return;
    }
    rtc_gpio_init((gpio_num_t)gpio_num);
    rtc_gpio_set_direction((gpio_num_t)gpio_num, RTC_GPIO_MODE_DISABLED);
    rtc_gpio_pullup_dis((gpio_num_t)gpio_num);
    rtc_gpio_pulldown_en((gpio_num_t)gpio_num);
}

static void unpark_pad(int gpio_num)
{
    if (!rtc_gpio_is_valid_gpio((gpio_num_t)gpio_num)) {
        return;
    }
    /* Drop the RTC pulls before handing the pad back to the GPIO matrix. */
    rtc_gpio_pulldown_dis((gpio_num_t)gpio_num);
    rtc_gpio_pullup_dis((gpio_num_t)gpio_num);
    rtc_gpio_deinit((gpio_num_t)gpio_num);
}

void display_prepare_deep_sleep(void)
{
    /* Waits out any repaint already in flight on the display task. */
    DISPLAY_LOCK();
    sleep_locked();

    /* RST is G18 on Plus SE (not the MTDI strap). BL is AXP LDO2. */
    park_pad_low(PIN_LCD_RST);
    s_parked = true;
    DISPLAY_UNLOCK();
}

esp_err_t display_resume(void)
{
    if (!s_ready) {
        return display_init();
    }

    DISPLAY_LOCK();
    if (s_parked) {
        /*
         * park_pad_low() moved both pads onto the RTC mux, so the digital
         * writes behind bl_set() stop reaching the pin. Hand them back before
         * touching the panel again, or the backlight can never come on.
         */
        unpark_pad(PIN_LCD_RST);
        s_parked = false;
    }

    /* The digital OUT/OE latches survived, so this is enough to drive it. */
    bl_set(false);

    esp_err_t ret = ESP_OK;
    if (s_panel != NULL) {
        ret = esp_lcd_panel_disp_on_off(s_panel, true);
    }
    s_drawn = -1;   /* force a repaint on the next display_set_status() */
    DISPLAY_UNLOCK();
    /* Caller redraws (and turns the backlight back on) via display_set_status(). */
    return ret;
}
