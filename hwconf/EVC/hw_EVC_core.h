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

#ifndef HW_EVC_CORE_H_
#define HW_EVC_CORE_H_

#ifdef HW_EVC_IS_REV1
#define HW_NAME					"EVC_Controller REV1"
#else
#error "Must include hardware type"
#endif

#define HW_MAJOR				1
#define HW_MINOR				0

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_FILTERS
#define HW_HAS_PHASE_SHUNTS
#define HW_DEAD_TIME_NSEC		300

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 9)
#define DISABLE_GATE()			palClearPad(GPIOC, 9)
#define ENABLE_PRECHARGE()      palSetPad(GPIOE, 6)
#define DISABLE_PRECHARGE()     palClearPad(GPIOE, 6)
#define ENABLE_MAIN_COIL()      palSetPad(GPIOE, 5)
#define DISABLE_MAIN_COIL()     palClearPad(GPIOE, 5)
#define DCCAL_ON()
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 12)) // check if there a better flag to use for hardware fault

#define PHASE_FILTER_GPIO		GPIOE
#define PHASE_FILTER_PIN		7
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

#define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
#define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)

#define CLEAR_CURR_FAULT_GPIO    GPIOD
#define CLEAR_CURR_FAULT_PIN     7

#define HALL_FILTER_GPIO		GPIOC
#define HALL_FILTER_PIN			5
#define HALL_FILTER_ON()		palSetPad(HALL_FILTER_GPIO, HALL_FILTER_PIN)
#define HALL_FILTER_OFF()		palClearPad(HALL_FILTER_GPIO, HALL_FILTER_PIN)

#define LED_GREEN_GPIO			GPIOE
#define LED_GREEN_PIN			14
#define LED_RED_GPIO			GPIOE
#define LED_RED_PIN				15
#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

/*
 * ADC Vector jack fucked this up comment probably wrong
 *
 * 0:	IN10	CURR1
 * 1:	IN11	CURR2
 * 2:	IN12	CURR3
 * 3:	IN0		SENS1
 * 4:	IN1		SENS2
 * 5:	IN2		SENS3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)

// ADC Indexes
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
//#define ADC_IND_ADC_MUX			15
#define ADC_IND_VREFINT        12
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_TEMP_MOS_2		10
#define ADC_IND_TEMP_MOS_3		13


// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3 //TODO: maybe update using vrefint?
#endif
#ifndef VIN_R1
#define VIN_R1					250000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		0.7186 // output divider
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.002668 // effective with 2.5 mV/A after bypass shunt
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP_MOS1()		(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()		(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()		(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP(adc_ind)		hw_evc_get_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral (USART3 TX/RX on PD8/9)
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOD
#define HW_UART_TX_PIN			8
#define HW_UART_RX_PORT			GPIOD
#define HW_UART_RX_PIN			9

// Permanent UART Peripheral (SPI3/UART4 group)
// Defaulting to SPI3. Uncomment below to use as UART4.
/*
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11
*/

// SPI pins (SPI3/UART4 on PC10-12 and PA15) // comment out to use UART4 above
#define HW_SPI_DEV				SPID3
#define HW_SPI_GPIO_AF			GPIO_AF_SPI3
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			15
#define HW_SPI_PORT_SCK			GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI			12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO			11


// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOD
#define HW_ICU_PIN				12

// I2C Peripheral (TX/SCL/MOSI and RX/SDA/NSS on PB10/11)
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// CAN Peripheral (Main) - CAN H/L: originated from CANRX/TX on PD0/1
#define HW_CAN_DEV              CAND1
#define HW_CAN_RX_PORT          GPIOD
#define HW_CAN_RX_PIN           0
#define HW_CAN_TX_PORT          GPIOD
#define HW_CAN_TX_PIN           1
#define HW_CAN_GPIO_AF          GPIO_AF_CAN1

// Second onboard CAN connection (CAN TX/RX on PB5/6)
#define HW_HAS_DUAL_CAN
#define HW_CAN2_DEV             CAND2
#define HW_CAN2_RX_PORT         GPIOB
#define HW_CAN2_RX_PIN          5
#define HW_CAN2_TX_PORT         GPIOB
#define HW_CAN2_TX_PIN          6
#define HW_CAN2_GPIO_AF         GPIO_AF_CAN2

// IMU I2C (PB8/PB9) for BMI270 (using BMI160 driver macros)
#define BMI160_SDA_GPIO         GPIOB
#define BMI160_SDA_PIN          9
#define BMI160_SCL_GPIO         GPIOB
#define BMI160_SCL_PIN          8

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler


// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

float hw_evc_get_temp(void);

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			true	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

// Setting limits
#define HW_LIM_CURRENT			-120.0, 600.0
#define HW_LIM_CURRENT_IN		-120.0, 400.0
#define HW_LIM_CURRENT_ABS		0.0, 750.0
#define HW_LIM_VIN				6.0, 150.0
#define HW_LIM_VIN_MIN          6.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0
#define HW_PRECHARGE_DONE_RATE		5.0		// V/s

#endif /* HW_EVC_CORE_H_ */

