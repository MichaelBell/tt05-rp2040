#include <pico/stdlib.h>
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/dma.h"

#include "tt_pins.h"

static uint hstx_data;

void hstx_send_clocks(int num, int div)
{
    hstx_ctrl_hw->bit[CLK - 12] = 0x20000;
    hstx_ctrl_hw->csr = (div << HSTX_CTRL_CSR_CLKDIV_LSB) | (0 << HSTX_CTRL_CSR_CLKPHASE_LSB);
    
    gpio_set_function(CLK, GPIO_FUNC_HSTX);

    hstx_ctrl_hw->csr = (div << HSTX_CTRL_CSR_CLKDIV_LSB) | (0 << HSTX_CTRL_CSR_CLKPHASE_LSB) | HSTX_CTRL_CSR_EN_BITS;

    uint dma_ch = dma_claim_unused_channel(true);
    dma_channel_config c;
    c = dma_channel_get_default_config(dma_ch);
    channel_config_set_dreq(&c, DREQ_HSTX);
    channel_config_set_read_increment(&c, false);
    dma_channel_configure(
        dma_ch,
        &c,
        &hstx_fifo_hw->fifo,
        &hstx_data,
        num,
        true
    );

    dma_channel_wait_for_finish_blocking(dma_ch);
    dma_channel_unclaim(dma_ch);

    while (!(hstx_fifo_hw->stat & HSTX_FIFO_STAT_EMPTY_BITS))
		;

    sleep_us(100);
    hstx_ctrl_hw->csr = (1 << HSTX_CTRL_CSR_CLKDIV_LSB);

    gpio_set_function(CLK, GPIO_FUNC_SIO);
}
