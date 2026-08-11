#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include "pin_config.h"
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "SensorQMI8658.hpp"
#include "HWCDC.h"

HWCDC USBSerial;
static constexpr uint32_t LVGL_BUFFER_LINES = 20;
static constexpr uint8_t QMI8658_BOARD_ADDRESS = 0x6B;
static constexpr uint32_t SENSOR_UPDATE_INTERVAL_MS = 100;
static constexpr uint32_t SERIAL_LOG_INTERVAL_MS = 1000;
uint32_t screenWidth;
uint32_t screenHeight;
size_t draw_buffer_bytes;
lv_display_t *disp;
uint8_t *disp_draw_buf;
lv_obj_t *status_label;
lv_obj_t *chart;
lv_chart_series_t *acc_series_x;
lv_chart_series_t *acc_series_y;
lv_chart_series_t *acc_series_z;
bool lvgl_ready = false;
bool touch_ready = false;
bool qmi_ready = false;
SensorQMI8658 qmi;
IMUdata acc;
uint32_t last_sensor_update_ms = 0;
uint32_t last_serial_log_ms = 0;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 22, 0, 0, 0);
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) { if (FT3168) FT3168->IIC_Interrupt_Flag = true; }
uint32_t millis_cb(void) { return millis(); }
void my_disp_flush(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  if (gfx && area && px_map) gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, lv_area_get_width(area), lv_area_get_height(area));
  lv_disp_flush_ready(display);
}
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  LV_UNUSED(indev);
  data->state = LV_INDEV_STATE_REL;
  if (!touch_ready || !FT3168 || !FT3168->IIC_Interrupt_Flag) return;
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
    if (!touch_ready) { USBSerial.printf("FT3168 initialization failed (%u/5)\n", attempt); delay(500); }
  }
  if (touch_ready) FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE, FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
  else USBSerial.println("FT3168 unavailable; continuing without touch.");
  for (uint8_t attempt = 1; attempt <= 5 && !qmi_ready; ++attempt) {
    qmi_ready = qmi.begin(Wire, QMI8658_BOARD_ADDRESS, IIC_SDA, IIC_SCL);
    if (!qmi_ready) { USBSerial.printf("QMI8658 initialization failed (%u/5)\n", attempt); delay(500); }
  }
  if (qmi_ready) {
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();
    USBSerial.printf("QMI8658 ready at 0x%02X\n", QMI8658_BOARD_ADDRESS);
  } else USBSerial.println("QMI8658 unavailable; continuing without accelerometer.");
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
  status_label = lv_label_create(lv_scr_act());
  chart = lv_chart_create(lv_scr_act());
  if (!status_label || !chart) { USBSerial.println("LVGL object creation failed; LVGL disabled."); return; }
  lv_label_set_text(status_label, qmi_ready ? "QMI8658 ready (mg)" : "QMI8658 unavailable");
  lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 8);
  lv_obj_set_size(chart, 240, 250);
  lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -4000, 4000);
  lv_chart_set_point_count(chart, 20);
  acc_series_x = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
  acc_series_y = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
  acc_series_z = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  if (!acc_series_x || !acc_series_y || !acc_series_z) { USBSerial.println("LVGL chart series creation failed; LVGL disabled."); return; }
  lvgl_ready = true;
  USBSerial.println("Setup done");
}

void loop() {
  if (!lvgl_ready) { delay(1000); return; }
  lv_task_handler();
  const uint32_t now = millis();
  if (qmi_ready && (uint32_t)(now - last_sensor_update_ms) >= SENSOR_UPDATE_INTERVAL_MS) {
    last_sensor_update_ms = now;
    if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z) && chart && acc_series_x && acc_series_y && acc_series_z) {
      if ((uint32_t)(now - last_serial_log_ms) >= SERIAL_LOG_INTERVAL_MS) {
        last_serial_log_ms = now;
        USBSerial.printf("{ACCEL: %.3f,%.3f,%.3f}\n", acc.x, acc.y, acc.z);
      }
      lv_chart_set_next_value(chart, acc_series_x, (int32_t)lroundf(acc.x * 1000.0f));
      lv_chart_set_next_value(chart, acc_series_y, (int32_t)lroundf(acc.y * 1000.0f));
      lv_chart_set_next_value(chart, acc_series_z, (int32_t)lroundf(acc.z * 1000.0f));
    }
  }
  delay(5);
}
