#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "SensorPCF85063.hpp"
#include "HWCDC.h"

HWCDC USBSerial;
static constexpr uint32_t LVGL_BUFFER_LINES = 20;
uint32_t screenWidth;
uint32_t screenHeight;
size_t draw_buffer_bytes;
lv_display_t *disp;
uint8_t *disp_draw_buf;
lv_obj_t *label;
SensorPCF85063 rtc;
uint32_t lastMillis;
bool lvgl_ready = false;
bool touch_ready = false;
bool rtc_ready = false;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 22, 0, 0, 0);
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  if (FT3168) {
    FT3168->IIC_Interrupt_Flag = true;
  }
}

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *buf) {
  LV_UNUSED(level);
  USBSerial.println(buf);
}
#endif

uint32_t millis_cb(void) { return millis(); }

void my_disp_flush(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  if (gfx && area && px_map) {
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map,
                            lv_area_get_width(area), lv_area_get_height(area));
  }
  lv_disp_flush_ready(display);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  LV_UNUSED(indev);
  data->state = LV_INDEV_STATE_REL;
  if (!touch_ready || !FT3168 || !FT3168->IIC_Interrupt_Flag) {
    return;
  }
  FT3168->IIC_Interrupt_Flag = false;
  data->point.x = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  data->point.y = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  data->state = LV_INDEV_STATE_PR;
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
  if (!gfx || !gfx->begin()) {
    USBSerial.println("Display initialization failed; LVGL disabled.");
    return;
  }
  gfx->fillScreen(RGB565_BLACK);
  Wire.begin(IIC_SDA, IIC_SCL);

  for (uint8_t attempt = 1; FT3168 && attempt <= 5 && !touch_ready; ++attempt) {
    touch_ready = FT3168->begin();
    if (!touch_ready) {
      USBSerial.printf("FT3168 initialization failed (%u/5)\n", attempt);
      delay(500);
    }
  }
  if (touch_ready) {
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
  } else {
    USBSerial.println("FT3168 unavailable; continuing without touch.");
  }

  rtc_ready = rtc.begin(Wire, IIC_SDA, IIC_SCL);
  if (!rtc_ready) {
    USBSerial.println("RTC unavailable; continuing without clock updates.");
  }

  lv_init();
  lv_tick_set_cb(millis_cb);
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif
  screenWidth = gfx->width();
  screenHeight = gfx->height();
  draw_buffer_bytes = (size_t)screenWidth * LVGL_BUFFER_LINES * lv_color_format_get_size(LV_COLOR_FORMAT_RGB565);
  disp_draw_buf = (uint8_t *)heap_caps_malloc(draw_buffer_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (uint8_t *)malloc(draw_buffer_bytes);
  if (!disp_draw_buf) {
    USBSerial.println("LVGL buffer allocation failed; LVGL disabled.");
    return;
  }
  disp = lv_display_create(screenWidth, screenHeight);
  if (!disp) {
    USBSerial.println("LVGL display creation failed; LVGL disabled.");
    return;
  }
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, disp_draw_buf, NULL, draw_buffer_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_add_event_cb(disp, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
  if (touch_ready) {
    lv_indev_t *indev = lv_indev_create();
    if (indev) {
      lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
      lv_indev_set_read_cb(indev, my_touchpad_read);
    } else {
      touch_ready = false;
      USBSerial.println("LVGL touch input creation failed; continuing without touch.");
    }
  }
  label = lv_label_create(lv_scr_act());
  if (!label) {
    USBSerial.println("LVGL label creation failed; LVGL disabled.");
    return;
  }
  lv_label_set_text(label, rtc_ready ? "Initializing clock..." : "RTC unavailable");
  lv_obj_set_style_text_font(label, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lvgl_ready = true;
  USBSerial.println("Setup done");
}

void loop() {
  if (!lvgl_ready) {
    delay(1000);
    return;
  }
  lv_task_handler();
  if (rtc_ready && label && millis() - lastMillis > 1000) {
    lastMillis = millis();
    RTC_DateTime datetime = rtc.getDateTime();
    char timeText[32];
    snprintf(timeText, sizeof(timeText), "%02d:%02d:%02d\n%02d-%02d-%04d", datetime.getHour(), datetime.getMinute(), datetime.getSecond(), datetime.getDay(), datetime.getMonth(), datetime.getYear());
    lv_label_set_text(label, timeText);
  }
  delay(5);
}
