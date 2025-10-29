/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "systems/phone/widgets/app_launcher/esp_brookesia_app_launcher.hpp"

namespace esp_brookesia::systems::phone {

constexpr AppLauncherIcon::Data STYLESHEET_DEFAULT_DARK_APP_LAUNCHER_ICON_DATA = {
    .main = {
        .size = gui::StyleSize::SQUARE(140),
        .layout_row_pad = 10,
    },
    .image = {
        .default_size = gui::StyleSize::SQUARE(98),
        .press_size = gui::StyleSize::SQUARE(88),
    },
    .label = {
        .text_font = gui::StyleFont::SIZE(16),
        .text_color = gui::StyleColor::COLOR(0xFFFFFF),
    },
};

constexpr AppLauncherData STYLESHEET_DEFAULT_DARK_APP_LAUNCHER_DATA = {
    .main = {
        .y_start = 0,
        .size = gui::StyleSize::RECT_PERCENT(100, 100),
    },
    .table = {
        .default_num = 3,
        .size = gui::StyleSize::RECT_PERCENT(100, 70),
    },
    .indicator = {
        .main_size = gui::StyleSize::RECT_W_PERCENT(100, 20),
        .main_layout_column_pad = 10,
        .main_layout_bottom_offset = 30,
        .spot_inactive_size = gui::StyleSize::SQUARE(12),
        .spot_active_size = gui::StyleSize::RECT(40, 12),
        .spot_inactive_background_color = gui::StyleColor::COLOR(0xC6C6C6),
        .spot_active_background_color = gui::StyleColor::COLOR(0xFFFFFF),
    },
    .icon = STYLESHEET_DEFAULT_DARK_APP_LAUNCHER_ICON_DATA,
    .flags = {
        .enable_table_scroll_anim = 0,
    },
};

} // namespace esp_brookesia::systems::phone
