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

static void __not_in_flash_func(core1_main)() {
    // Start 6.25MHz clock = 200/32
    gpio_set_function(0, GPIO_FUNC_PWM);

    // Set period of 20 cycles (0 to 19 inclusive)
    pwm_set_wrap(0, DIVIDER-1);
    // Set channel A output high for ten cycles before dropping
    pwm_set_chan_level(0, PWM_CHAN_A, DIVIDER/2);
    // Set the PWM running
    pwm_set_enabled(0, true);

  uint8_t memory[64][16] = { 0 };
  uint8_t addr_hi = 0;
  uint8_t addr_lo = 0;

  memset(memory, 0x0, 64 * 16);

  for (int i = 0; i < 64; ++i) {
    memory[i][0] = 0x4;
    memory[i][1] = 0x5;
    memory[i][2] = 0x6;
  }

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

const uint CAPTURE_PIN_BASE = 14;
const uint CAPTURE_PIN_COUNT = 15;
const uint CAPTURE_N_SAMPLES = 200;

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(pin_count));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    printf("Capture:\n");
    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (uint pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (uint32_t sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "-" : "_");
        }
        printf("\n");
    }
}

void capture() {

    // We're going to capture into a u32 buffer, for best DMA efficiency. Need
    // to be careful of rounding in case the number of pins being sampled
    // isn't a power of 2.
    uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT;
    total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT) - 1;
    uint buf_size_words = total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT);
    uint32_t *capture_buf = (uint32_t*)malloc(buf_size_words * sizeof(uint32_t));
    hard_assert(capture_buf);

    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
    //bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    PIO pio = pio0;
    uint sm = 0;
    uint dma_chan = 0;

    logic_analyser_init(pio, sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, DIVIDER/2);

    printf("Arming trigger\n");
    logic_analyser_arm(pio, sm, dma_chan, capture_buf, buf_size_words, 28, false);

    multicore_launch_core1(core1_main);

    // The logic analyser should have started capturing as soon as it saw the
    // first transition. Wait until the last sample comes in from the DMA.
    dma_channel_wait_for_finish_blocking(dma_chan);

    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
	sleep_ms(10);
    set_sys_clock_khz(300000, true);
	sleep_ms(10);

    stdio_init_all();

    // Uncomment to pause until the USB is connected before continuing
    //while (!stdio_usb_connected());

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

    //capture();

    while (1) __wfe();
}
