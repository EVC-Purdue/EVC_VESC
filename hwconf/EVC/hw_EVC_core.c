/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

// EVC woooooooo

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "drv8301.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

typedef enum {
	BOOTED = 0,
	PRECHARGING,
	PRECHARGED,
	PRECHARGE_FAILED
} precharge_state_t;

static volatile precharge_state_t precharge_state = BOOTED;

static THD_WORKING_AREA(precharge_thread_wa, 128);
static THD_FUNCTION(precharge_thread, arg);

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ENABLE_GATE
	palSetPadMode(GPIOC, 9,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// PRECHARGE and CONTACTOR
	palSetPadMode(GPIOE, 5,
			PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOE, 6,
			PAL_MODE_OUTPUT_PUSHPULL);
	
	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	CURRENT_FILTER_OFF();

	// Clear curr fault detect
	palSetPadMode(GPIOD, 7, PAL_MODE_INPUT);

	// Hall filter
	palSetPadMode(HALL_FILTER_GPIO, HALL_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	HALL_FILTER_ON();

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	// GPIOA/B Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors. If EVC Logic board has 5V pullups populated, MUST be HI-Z
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT);

	// Fault pin
	palSetPadMode(GPIOB, 12, PAL_MODE_INPUT);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // P1V
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // P2V
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // P3V
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // MOS1 temp
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // ADC1 EXT
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // ADC2 EXT

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG); // MOS2 temp
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG); // MOS3 temp

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // p1I
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // p2I
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // p3I
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); // VBAT
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // MOT temp

}


static THD_FUNCTION(precharge_thread, arg) {
	(void)arg;
	chRegSetThreadName("precharge_thread");

	while (precharge_state == BOOTED || precharge_state == PRECHARGING) {
		switch (precharge_state) {
		case BOOTED:
			precharge_state = PRECHARGING;
			break;

		case PRECHARGING:
			mc_interface_set_current(0);
			mc_interface_lock();
			DISABLE_GATE();
			int cts = 0;
			//check if ADCS are active and working
			while((ADC_Value[ADC_IND_VIN_SENS] < 1) && (cts < 50)){
				chThdSleepMilliseconds(100);
				cts++;
			}
			if (GET_INPUT_VOLTAGE() > 20.0) {
				precharge_state = PRECHARGE_FAILED;
				break;
			}
			ENABLE_PRECHARGE();
			cts = 0;
			float rate = 100.0;
			float last_voltage = GET_INPUT_VOLTAGE();
			//Wait for precharge resistors to precharge bulk caps
			while(((GET_INPUT_VOLTAGE() < HW_LIM_VIN_MIN) || (rate > HW_PRECHARGE_DONE_RATE)) && (cts < 50)){
				rate = ((GET_INPUT_VOLTAGE() - last_voltage) / 0.1);
				chThdSleepMilliseconds(100);
				last_voltage = GET_INPUT_VOLTAGE();
				cts++;
			}
			if (cts >= 50) {
				precharge_state = PRECHARGE_FAILED;
				DISABLE_PRECHARGE();
				break;
			}
			ENABLE_MAIN_COIL();
			chThdSleepMilliseconds(100);
			DISABLE_PRECHARGE();
			chThdSleepMilliseconds(100);

			//confirm contactor close
			if (GET_INPUT_VOLTAGE() < last_voltage) {
				precharge_state = PRECHARGE_FAILED;
				DISABLE_MAIN_COIL();
				break;
			}

			mc_interface_unlock();
			ENABLE_GATE();
			precharge_state = PRECHARGED;
			break;
		}
	}
}


void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp); // p1I index 0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, t_samp); //p1V index 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, t_samp); // ADC1 EXT index 6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp); // mot tmp index 9
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp); // Vref index 12

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, t_samp); // p2I index 1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, t_samp); // p2V index 4
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, t_samp); // ADC2 EXT index 7
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 4, t_samp); // MOS2 temp index 10
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 5, t_samp); // MOS3 temp index 13

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp); // p3I index 2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, t_samp); // p3V index 5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, t_samp); // MOS1 tmp index 8
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp); // VBAT index 11
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 5, t_samp); // p2I duplicate for alignment index 14

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp); // p1I
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp); // p2I
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp); // p3I
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp); // p1I
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp); // p2I
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp); // p3I
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp); // p1I
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp); // p2I
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp); // p3I

	chThdCreateStatic(precharge_thread_wa, sizeof(precharge_thread_wa),
			NORMALPRIO, precharge_thread, NULL);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}
