#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "board_touch.h"
#include "lv_conf.h"
#include "HWCDC.h"

HWCDC USBSerial;
static constexpr uint32_t LVGL_BUFFER_LINES = 20;
uint32_t screenWidth;
uint32_t screenHeight;
size_t draw_buffer_bytes;
lv_display_t *disp;
uint8_t *disp_draw_buf;
lv_obj_t *uptime_label;
bool lvgl_ready = false;
BoardTouch touch;
bool touch_ready = false;
uint32_t last_uptime_update;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 22, 0, 0, 0);
uint32_t millis_cb(void) { return millis(); }
void my_disp_flush(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  if (gfx && area && px_map) gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, lv_area_get_width(area), lv_area_get_height(area));
  lv_disp_flush_ready(display);
}
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  LV_UNUSED(indev);
  int16_t x, y;
  if (touch.readPoint(x, y)) {
    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}
void rounder_event_cb(lv_event_t *e) {
  lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
  if (!area || screenWidth == 0 || screenHeight == 0) return;
  int32_t y1 = area->y1 & ~1;
  int32_t y2 = area->y2 | 1;
  area->x1 = 0;
  area->x2 = (int32_t)screenWidth - 1;
  area->y1 = y1 < 0 ? 0 : y1;
  area->y2 = y2 >= (int32_t)screenHeight ? (int32_t)screenHeight - 1 : y2;
}

void setup() {
#ifdef DEV_DEVICE_INIT
  DEV_DEVICE_INIT();
#endif
  USBSerial.begin(115200);
  if (!gfx || !gfx->begin()) { USBSerial.println("Display initialization failed; LVGL disabled."); return; }
  gfx->fillScreen(RGB565_BLACK);
  const char *controller = nullptr;
  for (uint8_t attempt = 1; attempt <= 5 && !controller; ++attempt) {
    controller = touch.begin();
    if (!controller && attempt < 5) delay(500);
  }
  touch_ready = controller != nullptr;
  if (touch_ready) {
    USBSerial.printf("Touch controller: %s\n", controller);
  } else {
    USBSerial.println("Touch unavailable; continuing without touch.");
  }
  lv_init();
  lv_tick_set_cb(millis_cb);
  screenWidth = gfx->width();
  screenHeight = gfx->height();
  draw_buffer_bytes = (size_t)screenWidth * LVGL_BUFFER_LINES * lv_color_format_get_size(LV_COLOR_FORMAT_RGB565);
  disp_draw_buf = (uint8_t *)heap_caps_malloc(draw_buffer_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (uint8_t *)malloc(draw_buffer_bytes);
  if (!disp_draw_buf) { USBSerial.println("LVGL buffer allocation failed; LVGL disabled."); return; }
  disp = lv_display_create(screenWidth, screenHeight);
  if (!disp) { USBSerial.println("LVGL display creation failed; LVGL disabled."); return; }
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, disp_draw_buf, NULL, draw_buffer_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_add_event_cb(disp, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
  if (touch_ready) {
    lv_indev_t *indev = lv_indev_create();
    if (indev) { lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); lv_indev_set_read_cb(indev, my_touchpad_read); }
    else { touch_ready = false; USBSerial.println("LVGL touch input creation failed; continuing without touch."); }
  }
  lv_obj_t *title = lv_label_create(lv_scr_act());
  uptime_label = lv_label_create(lv_scr_act());
  if (!title || !uptime_label) { USBSerial.println("LVGL object creation failed; LVGL disabled."); return; }
  lv_label_set_text(title, "LVGL status");
  lv_obj_align(title, LV_ALIGN_CENTER, 0, -20);
  lv_label_set_text(uptime_label, "Uptime: 0 s");
  lv_obj_align(uptime_label, LV_ALIGN_CENTER, 0, 20);
  lvgl_ready = true;
  USBSerial.println("Setup done");
}

void loop() {
  if (!lvgl_ready) { delay(1000); return; }
  lv_task_handler();
  if (uptime_label && millis() - last_uptime_update >= 1000) {
    last_uptime_update = millis();
    char text[32];
    snprintf(text, sizeof(text), "Uptime: %lu s", (unsigned long)(last_uptime_update / 1000));
    lv_label_set_text(uptime_label, text);
  }
  delay(5);
}
