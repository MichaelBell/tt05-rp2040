#include <pico/stdlib.h>
#include <hardware/gpio.h>

#include "tt_pins.h"

void tt_select_design(int idx)
{
    // Ensure all pins are set to safe defaults.
    gpio_set_dir_all_bits(0);
    gpio_init_mask(0x3FFFFFFF);

    // Set CLK, MUX sel, enable and inc low, reset high
    gpio_put_all((1 << SDI_nRST) | (1 << nCRST_OUT2));

    // Enable non-muxed signals, this sets the mux dir
    gpio_set_dir_all_bits((1 << CLK) | (1 << HK_CSB));
    sleep_us(10);

    // Enable muxed signals
    gpio_set_dir_all_bits((1 << CLK) | (1 << HK_CSB) | (1 << CTRL_ENA_OUT1) | (1 << nCRST_OUT2) | (1 << CINC_OUT3));
    sleep_us(10);

    // Mux control reset
    gpio_put(nCRST_OUT2, 0);
    sleep_us(100);
    gpio_put(nCRST_OUT2, 1);
    sleep_us(100);

    // Mux select
    for (int i = 0; i < idx; ++i) {
        gpio_put(CINC_OUT3, 1);
        sleep_us(10);
        gpio_put(CINC_OUT3, 0);
        sleep_us(10);
    }

    // Enable design
    sleep_us(20);
    gpio_put(CTRL_ENA_OUT1, 1);
    sleep_us(20);

    // Set pull ups on CENA, nCRST / CINC and change all muxed pins to inputs
    gpio_set_pulls(CTRL_ENA_OUT1, true, false);
    gpio_set_pulls(nCRST_OUT2, true, false);
    gpio_set_pulls(CINC_OUT3, true, false);
    gpio_set_dir_all_bits((1 << CLK) | (1 << HK_CSB));
    sleep_us(10);

    // Switch mux
    gpio_put(HK_CSB, 1);
    sleep_us(10);

    // Set reset and project inputs to RP2040 outputs
    gpio_set_dir_all_bits((1 << CLK) | (1 << HK_CSB) | (1 << SDI_nRST) |
                          (0xF << IN0) | (0xF << IN4));

    // Leave design in reset
    gpio_put(SDI_nRST, 0);
    sleep_us(10);
}