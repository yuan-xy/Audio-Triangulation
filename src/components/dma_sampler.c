#include <components/dma_sampler.h>

volatile uint8_t dma_sample_array[3] = {0};

// Pointer array holding the reload address for ping-pong
static uint8_t *reload_ptr[1] = {dma_sample_array};

void dma_sampler_init(void)
{
    // Initialize ADC and GPIOs
    adc_init();
    adc_gpio_init(26 + MIC_A_ADC_CH);
    adc_gpio_init(26 + MIC_B_ADC_CH);
    adc_gpio_init(26 + MIC_C_ADC_CH);
    adc_set_temp_sensor_enabled(false);
    adc_select_input(MIC_A_ADC_CH);
    adc_set_round_robin((1u << MIC_A_ADC_CH) |
                        (1u << MIC_B_ADC_CH) |
                        (1u << MIC_C_ADC_CH));
    adc_fifo_setup(true, true, 1, false, true);
    adc_fifo_drain();
    adc_set_clkdiv(0);
    adc_run(true);

    int sample_chan = dma_claim_unused_channel(true);
    int ctrl_chan = dma_claim_unused_channel(true);

    // Configure sample channel (one-shot, triggers control channel)
    dma_channel_config samp_conf = dma_channel_get_default_config(sample_chan);
    channel_config_set_transfer_data_size(&samp_conf, DMA_SIZE_8);
    channel_config_set_read_increment(&samp_conf, false);
    channel_config_set_write_increment(&samp_conf, true);
    channel_config_set_dreq(&samp_conf, DREQ_ADC);
    channel_config_set_chain_to(&samp_conf, ctrl_chan);
    dma_channel_configure(sample_chan, &samp_conf,
                          dma_sample_array,
                          &adc_hw->fifo,
                          3,
                          false);

    // Configure control channel (writes reload address, retriggers sample)
    dma_channel_config ctrl_conf = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&ctrl_conf, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_conf, false);
    channel_config_set_write_increment(&ctrl_conf, false);
    channel_config_set_dreq(&ctrl_conf, DREQ_FORCE);
    dma_channel_configure(ctrl_chan, &ctrl_conf,
                          &dma_hw->ch[sample_chan].al2_write_addr_trig,
                          reload_ptr,
                          1,
                          false);

    // Start the chain
    dma_channel_start(ctrl_chan);
    dma_channel_start(sample_chan);
}
