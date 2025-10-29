/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <memory>
#include <cstdlib>
#include "lvgl.h"
#include "style/esp_brookesia_gui_style.hpp"
#include "esp_brookesia_lv_object.hpp"

namespace esp_brookesia::gui {

class LvCanvas: public LvObject {
public:
    LvCanvas(const LvObject *parent);

    bool setBuffer(void *buffer, int width, int height);
};

using LvCanvasUniquePtr = std::unique_ptr<LvCanvas>;

} // namespace esp_brookesia::gui
