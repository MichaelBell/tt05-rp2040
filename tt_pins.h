#pragma once

#include "hardware/gpio.h"

// Mapping for PGA2350-TT06 with broken TT05 carrier converter
enum GPIOMap {
    CLK = 0,
    nRST = 1,
    CENA = 4,
    nCRST = 2,
    CINC = 3,
    IN0 = 9,
    IN1 = 10,
    IN2 = 11,
    IN3 = 12,
    IN4  = 17,
    IN5  = 18,
    IN6  = 19,
    IN7  = 20,
    OUT0 = 5,
    OUT1 = 6,
    OUT2 = 7,
    OUT3 = 8,
    OUT4 = 13,
    OUT5 = 14,
    OUT6 = 15,
    OUT7 = 16,
    UIO0 = 21,
    UIO1 = 22,
    UIO2 = 23,
    UIO3 = 24,
    UIO4 = 25,
    UIO5 = 26,
    UIO6 = 27,
    UIO7 = 28,
};

inline static void tt_set_input_byte(int val) {
    val = (val << 4) | (val & 0xF);
    gpio_put_masked((0xF << IN0) | (0xF << IN4), val << IN0);
}

inline static int tt_get_output_byte() {
    int gpio = gpio_get_all();
    return ((gpio >> (OUT4 - 4)) & 0xF0) | ((gpio >> OUT0) & 0xF);
}

inline static void tt_clock_project_once() {
    gpio_xor_mask(1 << CLK);
    sleep_us(20);
    gpio_xor_mask(1 << CLK);
    sleep_us(20);
}