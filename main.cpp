#include <pico/stdlib.h>
#include <hardware/sync.h>
#include <pico/multicore.h>
#include <hardware/pwm.h>
#include "hardware/vreg.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "tt_setup.h"
#include "tt_pins.h"

#define DESIGN_NUM 204

#define MEM_CLK_PIN   28
#define MEM_START_PIN 14
#define MEM_CONT_PIN  16
#define MEM_WE_PIN    15

#define DIVIDER 48

static  uint8_t memory[64][16] = { 0 };

static void __not_in_flash_func(core1_main)() {
    // Start 6.25MHz clock = 200/32
    gpio_set_function(0, GPIO_FUNC_PWM);

    // Set period of 20 cycles (0 to 19 inclusive)
    pwm_set_wrap(0, DIVIDER-1);
    // Set channel A output high for ten cycles before dropping
    pwm_set_chan_level(0, PWM_CHAN_A, DIVIDER/2);
    // Set the PWM running
    pwm_set_enabled(0, true);

  uint8_t addr_hi = 0;
  uint8_t addr_lo = 0;

  memset(memory, 0x0, 64 * 16);

  /*
  for (int i = 0; i < 64; ++i) {
    memory[i][0] = 0x4;
    memory[i][1] = 0x5;
    memory[i][2] = 0x6;
  }*/

  /*memory[0][5] = 0x0;
  memory[10][5] = 0x0;*/

  memory[5][7] = 0x3;

  //int i = 0;

  bool prev_clk = false;
  bool writing = false;

  //int num_accesses = 0;

  //uint8_t report = 0xFF;

  while (1) {
    bool clk = gpio_get(MEM_CLK_PIN);

    if (!prev_clk && clk) {
      gpio_set_dir_in_masked(0x7 << UIO0);
      if (gpio_get(MEM_START_PIN)) {
        writing = gpio_get(MEM_WE_PIN);
        uint32_t psr = gpio_get_all();
        addr_hi = ((psr >> UIO0) & 0x3F);
        addr_lo = 0;

        /*if (num_accesses >= 16) {
          num_accesses = 0;

          if (report != 0xFF) {
            Serial.write(0xFF);
            Serial.write(report);
            for (int i = 0; i < 16; ++i) {
              Serial.write(memory[report][i]);
            }
          }
          report = 0xFF;
        }*/

        /*if (num_accesses == 0 || num_accesses == 1) {
          Serial.write((uint8_t)addr_hi);
        }

        ++num_accesses;
        if (num_accesses == 32) {
          num_accesses = 0;
        }*/

        /*if (!writing) {
          Serial.write(num_pulses);
          num_pulses = 0;
        }*/

/*
       putc('\n', stdout);
       putc('0' + addr_hi, stdout);
       if (writing) putc('w', stdout);*/

      } else if (gpio_get(MEM_CONT_PIN)) {
        if (writing) {
          uint32_t psr = gpio_get_all();
          memory[addr_hi][addr_lo] = ((psr >> UIO0) & 0x7);

          /*++num_accesses;
          if (memory[addr_hi][addr_lo] != 0) {
            report = addr_hi;
          }*/
        } else {
          uint32_t mem = ((addr_hi & 0x1F) < 20) ? memory[addr_hi][addr_lo] : 0x00;
          gpio_put_masked(0x7 << UIO0, mem << UIO0);
          gpio_set_dir_out_masked(0x7 << UIO0);
        }

        ++addr_lo;
        addr_lo &= 0xF;

        //putc('.', stdout);
      }
    }
    prev_clk = clk;

    /*
    ++i;
    if (i == 1000000) {
      i = 0;
      //Serial.println(addr_hi);
      i = 0;
      for (int j = 18; j < 20; ++j) {
        bool row_present = false;
        for (int k = 0; k < 16; ++k) {
          if (memory[j][k] != 0) {
            row_present = true;
            break;
          }
        }

        if (row_present) {
          Serial.println(j);
        }
      }
    }
    */
  }    

}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
	sleep_ms(10);
    set_sys_clock_khz(300000, true);
	sleep_ms(10);

    stdio_init_all();

    // If a button is pressed while waiting for USB connection,
    // then don't take control of inputs
    bool usb_control = true;
    while (!stdio_usb_connected()) {
        if (gpio_get_all() & ((1 << IN4) | (0xF << IN0))) {
            usb_control = false;
            break;
        }
    }

    sleep_ms(20);
    printf("Selecting Tetris\n");
    sleep_ms(10);

    tt_select_design(DESIGN_NUM);
    printf("Selected\n");
    sleep_ms(10);

    // Clock a few times in reset and then take out of reset
    tt_clock_project_once();
    tt_clock_project_once();
    tt_clock_project_once();
    gpio_put(SDI_nRST, 1);

    // Run on core 1 to avoid interrupts as timing is critical
    multicore_launch_core1(core1_main);

    if (usb_control) {
        gpio_set_dir_out_masked((1 << IN4) | (0xF << IN0));
        while (1) {
            int c = getchar();
            switch (c) {
                case 'a': tt_set_input_byte(1); break;
                case 's': tt_set_input_byte(4); break;
                case 'd': tt_set_input_byte(2); break;
                case 'q': case 'w': tt_set_input_byte(8); break;
                case 'e': tt_set_input_byte(16); break;
            }
            sleep_us(2000);
            tt_set_input_byte(0);
        }
    }
    else {
        while (1) __wfe();
    }

}
