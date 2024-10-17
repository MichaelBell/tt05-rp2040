#pragma once

#include "hardware/gpio.h"

// Mapping for PGA2350-TT06 with broken TT05 carrier converter
enum GPIOMap {
    CLK = 17,
    nRST = 20,
    CENA = 29,
    nCRST = 30,
    CINC = 31,
    IN0 = 1,
    IN1 = 2,
    IN2 = 3,
    IN3 = 4,
    IN4  = 5,
    IN5  = 6,
    IN6  = 7,
    IN7  = 8,
    OUT0 = 9,
    OUT1 = 10,
    OUT2 = 11,
    OUT3 = 12,
    OUT4 = 13,
    OUT5 = 14,
    OUT6 = 15,
    OUT7 = 16,
    UIO0 = 28,
    UIO1 = 27,
    UIO2 = 26,
    UIO3 = 25,
    UIO4 = 24,
    UIO5 = 23,
    UIO6 = 22,
    UIO7 = 21,
};

inline static void tt_set_input_byte(int val) {
    gpio_put_masked(0xFF << IN0, val << IN0);
}

inline static int tt_get_output_byte() {
    int gpio = gpio_get_all();
    return (gpio >> OUT0) & 0xFF;
}

inline static void tt_clock_project_once() {
    gpio_xor_mask(1 << CLK);
    sleep_us(20);
    gpio_xor_mask(1 << CLK);
    sleep_us(20);
}