#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <pico/multicore.h>

#include <array>
#include <cstdio>

#include "tt_setup.h"
#include "tt_pins.h"

#define DESIGN_NUM 1

int main() {
    set_sys_clock_khz(150000, true);

    stdio_init_all();

    // Uncomment to pause until the USB is connected before continuing
    while (!stdio_usb_connected());

    sleep_ms(20);
    printf("Selecting Factory Test\n");
    sleep_ms(10);

    tt_select_design(DESIGN_NUM);
    printf("Selected\n");
    sleep_ms(10);

    // Clock in reset and then take out of reset
    tt_clock_project_once();
    gpio_put(nRST, 1);

    // Factory test counts if in0 is high
    gpio_put(IN0, 1);

    while (1) {
        tt_clock_project_once();
        sleep_ms(100);
        printf("%d\n", tt_get_output_byte());
    }
}