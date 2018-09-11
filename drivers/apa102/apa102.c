/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_apa102
 * @{
 *
 * @file
 * @brief       APA 102 RGB LED driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <string.h>

#include "assert.h"
#include "apa102.h"

#define START           (0x00000000)
#define END             (0xffffffff)

#define HEAD            (0xe0000000)
#define BRIGHT_SHIFT    (21U)
#define BRIGHT_MASK     (0x1f000000)

#ifndef APA102_SHIFTED_COLORS
/* Standard LED colours, as per data sheet */
#define BLUE_SHIFT      (16U)
#define GREEN_SHIFT     (8U)
#define RED_SHIFT       (0U)
#else
/* Wrong LED colours */
#define RED_SHIFT       (16U)
#define BLUE_SHIFT      (8U)
#define GREEN_SHIFT     (0U)
#endif

static inline void shift(const apa102_t *dev, uint32_t data)
{
    for (int i = 31; i >= 0; i--) {
        gpio_write(dev->data_pin, ((data >> i) & 0x01));
        gpio_set(dev->clk_pin);
        gpio_clear(dev->clk_pin);
    }
}

void apa102_init(apa102_t *dev, const apa102_params_t *params)
{
    assert(dev && params);

    memcpy(dev, params, sizeof(apa102_params_t));

    gpio_init(dev->data_pin, GPIO_OUT);
    gpio_init(dev->clk_pin, GPIO_OUT);
    gpio_clear(dev->data_pin);
    gpio_clear(dev->clk_pin);
}

void apa102_load_rgba(const apa102_t *dev, const color_rgba_t vals[])
{
    assert(dev && vals);

    shift(dev, START);
    for (int i = 0; i < dev->led_numof; i++) {
        uint32_t data = HEAD;
        /* we scale the 8-bit alpha value to a 5-bit value by cutting off the
         * 3 leas significant bits */
        data |= (((uint32_t)vals[i].alpha << BRIGHT_SHIFT) & BRIGHT_MASK);
        data |=  ((uint32_t)vals[i].color.b << BLUE_SHIFT);
        data |=  ((uint32_t)vals[i].color.g << GREEN_SHIFT);
        data |=  ((uint32_t)vals[i].color.r << RED_SHIFT);
        shift(dev, data);
    }
    shift(dev, END);
}
