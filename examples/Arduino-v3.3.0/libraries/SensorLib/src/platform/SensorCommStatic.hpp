/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorCommStatic.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-21
 *
 */
#pragma once

#include "SensorCommBase.hpp"

class SensorCommStatic
{
public:
    SensorCommStatic(SensorCommBase *comm, SensorHal *hal)
        : comm(comm), hal(hal)
    {
    }
    static int8_t sensor_static_read_data(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static int8_t sensor_static_write_data(uint8_t reg_addr,const uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static void   sensor_static_delay_us(uint32_t us, void *private_data);
protected:
    SensorCommBase  *comm;
    SensorHal       *hal;
};



