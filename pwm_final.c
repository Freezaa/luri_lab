/* This source file is part of the ATMEL AVR-UC3-SoftwareFramework-1.7.0 Release */

/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief LCD DIP204 example driver for EVK1100 board.
 *
 * This file provides a useful example for the LCD DIP204 on SPI interface.
 * Press PB0 to see the full set of available chars on the LCD
 * Press PB1 to decrease the backlight power of the LCD
 * Press PB2 to increase the backlight power of the LCD
 * Use Joystick to see arrows displayed on the LCD
 * Press Joystick to return to the idle screen
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with : SPI and PWM
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/

/*! \page License
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */
/*! \mainpage
 * \section intro Introduction
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software. <BR>It also gives an example of the usage of the
 * DIP204 LCD on EVK1100. \n \n
 * <b>Example's operating mode:</b>
 * <ul>
 * <li>A default message is displayed on the 4 lines of the LCD
 * <li>Press PB0 to see the full set of available chars on the LCD
 * <li>Press PB1 to decrease the backlight power of the LCD
 * <li>Press PB2 to increase the backlight power of the LCD
 * <li>Use the joystick to see arrows displayed on the LCD
 * <li>Press the joystick to see a circle displayed on the LCD and to return to the
 *     idle screen (displaying the default message)
 *
 * </ul>
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC for AVR32 and IAR Systems compiler
 * for AVR32. Other compilers may or may not work.
 *
 * \section deviceinfo Device Info
 * All AVR32UC devices with an SPI module can be used. This example has been tested
 * with the following setup:
 *- EVK1100 evaluation kit
 *
 * \section setupinfo Setup Information
 * CPU speed: <i> 12 MHz </i>
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/products/AVR32/">Atmel AVR32</A>.\n
 * Support and FAQ: http://support.atmel.no/
 */


#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "gpio.h"
#include "pm.h"
#include "delay.h"
#include "spi.h"
#include "pwm.h"
#include <stdio.h>
#include <avr32/io.h>

#define MCK 12000000 // Master clock.
#define PWM_0 0
#define voltage 3.3
#define UPDATE_FAIL 1
#define UPDATE_SUCC 0

unsigned int dty_cycle = 6000;
unsigned int period = (MCK/2)/1000;
int freq_inc = 10;

avr32_pwm_channel_t pwm_channel = { .ccnt = 0 }; //PWM channel config.

int get_freq()
{
	return (MCK/2) / period;
}

int get_period()
{
	return period;
}

int get_duty()
{
	return dty_cycle;
}

float get_duty_per()
{
	return ((float)(period-dty_cycle)/(float)period);
}

int update_dty_cycle(float duty_percent)
{
	if(duty_percent > 1){
		duty_percent = 1;
	}
	if(duty_percent < 0){
		duty_percent = 0;
	}
	unsigned int new_dty_cycle = duty_percent*period;
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;
	pwm_channel.cupd = (new_dty_cycle); // Update Value
	pwm_async_update_channel(PWM_0, &pwm_channel);
	dty_cycle = new_dty_cycle;
	return UPDATE_SUCC;

}


void update_voltage(float voltage_new) {
	update_dty_cycle((voltage - voltage_new) / voltage);
}


int update_frequency(int freq)
{
	if(freq <= 0){
		return UPDATE_FAIL;
	}


	unsigned int new_period = (MCK/2)/freq;


    unsigned int new_dty_cycle2 = (float)new_period*(1 - get_duty_per());
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;
	pwm_channel.cupd = (new_dty_cycle2); // Update Value
	pwm_async_update_channel(PWM_0, &pwm_channel);
	dty_cycle = new_dty_cycle2;
	delay_ms(50);

	pwm_channel.CMR.cpd = PWM_UPDATE_PERIOD;
	pwm_channel.cupd = new_period; // Update Value
	pwm_async_update_channel(PWM_0, &pwm_channel);
	period = new_period;
	return UPDATE_SUCC;

}




/*!
 * \brief The Push Buttons interrupt handler.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_PB_int_handler(void)
{


  /* display all available chars */
  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_0))
  {
	update_frequency(get_freq() - freq_inc);
    gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_0);
  }
  /* increase backlight power */
  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_1))
  {
	update_frequency(get_freq() + freq_inc);
    gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_1);
  }
  /* decrease backlight power */
  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_2))
  {
	if(freq_inc < 1000){
		freq_inc *= 10;
	}
	else{
		freq_inc = 10;
	}
    gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_2);
  }
}


/*!
 * \brief The joystick interrupt handler.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_Joy_int_handler(void)
{
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP))
  {
	update_voltage(get_duty_per() * voltage + .1);
	gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
  }
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN))
  {
	update_voltage(get_duty_per() * voltage - .1);
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
  }
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_LEFT))
  {
	update_dty_cycle(1 - get_duty_per() - .01);
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_LEFT);
  }
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT))
  {
	update_dty_cycle(1 - get_duty_per() + .01);
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT);
  }
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_PUSH))
  {

    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
  }
}


/*!
 * \brief function to configure push button to generate IT upon rising edge
 */
void dip204_example_configure_push_buttons_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_2 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_1 , GPIO_RISING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_2/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_1/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_0/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}


/*!
 * \brief function to configure joystick to generate IT upon falling edge
 */
void dip204_example_configure_joystick_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_UP , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_DOWN , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_RIGHT , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_PUSH , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_LEFT , GPIO_FALLING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_UP/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_DOWN/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_RIGHT/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_LEFT/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_PUSH/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}

void pwm_initalize(void)
{

	gpio_enable_module_pin(AVR32_PIN_PB19, 0); //Assign GPIO pin to PWM function.
	pwm_opt_t pwm_opt; //PWM option config.

// PWM controller configuration.
	pwm_opt.diva = AVR32_PWM_DIVA_CLK_OFF;
	pwm_opt.divb = AVR32_PWM_DIVB_CLK_OFF;
	pwm_opt.prea = AVR32_PWM_PREA_MCK;
	pwm_opt.preb = AVR32_PWM_PREB_MCK;
	pwm_init(&pwm_opt);
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; //Channel mode.
	pwm_channel.CMR.cpol = PWM_POLARITY_LOW; // Channel polarity.
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_2; //Channel prescaler MCK/1.
	pwm_channel.cprd = period; // Channel period.
	pwm_channel.cdty = dty_cycle; //Channel duty cycle, 0 <= pwm_channel.cdty <= pwm_channel.cprd
	// with MCK == 12000000Hz, prescaler == 1, period == 12000, duty cycle == 12000, Channel polarity = low.
	// The output waveform period will be :
	// Freq =(MCK/prescaler)/period == (12000000/1)/(FPBA/1000) ==1KHz,D ==0%.
	pwm_channel_init(PWM_0,&pwm_channel); // Set channel configuration to channel 0.
	pwm_start_channels(1 << PWM_0); // Start channel 0.
}
/*!
 * \brief main function : do init and loop (poll if configured so)
 */
int main(void)
{


	static const gpio_map_t DIP204_SPI_GPIO_MAP =
  {
    {DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
    {DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
    {DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
    {DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
  };

  // Switch the CPU main clock to oscillator 0
  pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

  // Disable all interrupts.
  Disable_global_interrupt();

  // init the interrupts
  INTC_init_interrupts();

  // Enable all interrupts.
  Enable_global_interrupt();

  // add the spi options driver structure for the LCD DIP204
  spi_options_t spiOptions =
  {
    .reg          = DIP204_SPI_NPCS,
    .baudrate     = 1000000,
    .bits         = 8,
    .spck_delay   = 0,
    .trans_delay  = 0,
    .stay_act     = 1,
    .spi_mode     = 0,
    .modfdis      = 1
  };

  // Assign I/Os to SPI
  gpio_enable_module(DIP204_SPI_GPIO_MAP,
                     sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

  // Initialize as master
  spi_initMaster(DIP204_SPI, &spiOptions);

  // Set selection mode: variable_ps, pcs_decode, delay
  spi_selectionMode(DIP204_SPI, 0, 0, 0);

  // Enable SPI
  spi_enable(DIP204_SPI);

  // setup chip registers
  spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

  // configure local push buttons
  dip204_example_configure_push_buttons_IT();

  // configure local joystick
  dip204_example_configure_joystick_IT();

  // initialize delay driver
  delay_init( FOSC0 );

  // initialize LCD
  dip204_init(backlight_PWM, TRUE);

  pwm_initalize();

  char output[80];


  /* do a loop */
  while (1)
  {
	  dip204_clear_display();

      dip204_set_cursor_position(1,1);
      sprintf(output, "P: %d", period);
      dip204_write_string(output);

      dip204_set_cursor_position(10,1);
      sprintf(output, "dty: %d", get_duty());
      dip204_write_string(output);

      dip204_set_cursor_position(1,2);
      sprintf(output, "Duty: %0.3f", get_duty_per() *100);
      dip204_write_string(output);

      dip204_set_cursor_position(1,3);
      sprintf(output, "f: %d", get_freq());
      dip204_write_string(output);

      dip204_set_cursor_position(9,3);
      sprintf(output, "step: %d", freq_inc);
      dip204_write_string(output);

      dip204_set_cursor_position(1,4);
      sprintf(output, "v: %1.2f", get_duty_per()*(float)voltage);
      dip204_write_string(output);
      delay_ms(100);
	  }
  }

