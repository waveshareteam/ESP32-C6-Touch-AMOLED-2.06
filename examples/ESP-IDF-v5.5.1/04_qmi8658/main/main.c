#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "qmi8658.h"

static const char *TAG = "QMI8658_DEMO";

i2c_master_bus_handle_t bus_handle;
static lv_obj_t *label;

static void imu_display_task(void *arg)
{
    qmi8658_dev_t *dev = (qmi8658_dev_t *)arg;
    qmi8658_data_t data;

    char buf[128];

    while (1)
    {
        if (qmi8658_read_sensor_data(dev, &data) == ESP_OK)
        {
            snprintf(buf, sizeof(buf),
                     "QMI8658 Data:\n"
                     "AccelX: %.3f m/s\n"
                     "AccelY: %.3f m/s\n"
                     "AccelZ: %.3f m/s",
                     data.accelX, data.accelY, data.accelZ);

            bsp_display_lock(pdMS_TO_TICKS(100));
            lv_label_set_text(label, buf);
            bsp_display_unlock();
        }
        else
        {
            ESP_LOGW(TAG, "Failed to read sensor data");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    lv_display_t *disp = bsp_display_start();
    bsp_display_backlight_on();

    bsp_display_lock(pdMS_TO_TICKS(200));
    label = lv_label_create(lv_screen_active());
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(label, "Initializing QMI8658...");
    bsp_display_unlock();

    bus_handle = bsp_i2c_get_handle();

    static qmi8658_dev_t dev;
    ESP_ERROR_CHECK(qmi8658_init(&dev, bus_handle, QMI8658_ADDRESS_HIGH));

    qmi8658_set_accel_range(&dev, QMI8658_ACCEL_RANGE_8G);
    qmi8658_set_accel_odr(&dev, QMI8658_ACCEL_ODR_500HZ);
    qmi8658_set_accel_unit_mps2(&dev, true);
    qmi8658_write_register(&dev, QMI8658_CTRL5, 0x03);

    xTaskCreatePinnedToCore(
        imu_display_task,
        "imu_display_task",
        4096,
        &dev,
        3,
        NULL,
        0);
}
