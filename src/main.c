/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries

#include "pt_cornell_rp2040_v1_3.h"

// Some macros for max/min/abs
#define min(a,b) (((a)<(b)) ? (a):(b))
#define max(a,b) (((a)<(b)) ? (b):(a))
#define abs(a) (((a)>(0)) ? (a):-(a))

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;



static unsigned char buffer_index = 0;
static int buffer_a[256] = {};
static int buffer_b[256] = {};
static int buffer_c[256] = {};

void push(int a, int b, int c) {
    buffer_a[buffer_index] = a; 
    buffer_b[buffer_index] = b;
    buffer_c[buffer_index] = c;
    buffer_index += 1;
}

void sample_all_adcs() {
    // Read ADC0
    adc_select_input(0);
    uint16_t adc0 = adc_read();
    // Read ADC1
    adc_select_input(1);
    uint16_t adc1 = adc_read();
    // Read ADC2
    adc_select_input(2);
    uint16_t adc2 = adc_read();

    // Push the values to the buffer
    push(adc0, adc1, adc2);
}

void load_buffers() {
    absolute_time_t now;
    
    buffer_index = 0;
    now = get_absolute_time();
    for (int i = 0; i < 256; i++) {
        now += make_timeout_time_ms(20);
        busy_wait_until(now);
        sample_all_adcs();
    }

    int total_a,  total_b,  total_c;
    total_a = 0, total_b = 0, total_c = 0;

    for (int i = 0; i < 256; i++) {
        total_a += buffer_a[i];
        total_b += buffer_b[i];
        total_c += buffer_c[i];
    }

    total_a >>= 8;
    total_b >>= 8;
    total_c >>= 8;

    for (int i = 0; i < 256; i++) {
        buffer_a[i] -= total_a;
        buffer_b[i] -= total_b;
        buffer_c[i] -= total_c;
    }

}

bool buffer_juicy() {
    long power = 0;
    for (int i = 0; i < 256; i++) {
        power += (buffer_a[i]) * (buffer_a[i]);
        power += (buffer_b[i]) * (buffer_b[i]);
        power += (buffer_c[i]) * (buffer_c[i]);
    }
    long average_power = power / 768;
    return (average_power >= 1024);
}

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
 
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();
    adc_init();
    adc_gpio_init(26); // ADC0
    adc_gpio_init(27); // ADC1
    adc_gpio_init(28); // ADC2

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    while(1) {
        load_buffers();

        if (buffer_juicy()) {
            
        }
    }

}
