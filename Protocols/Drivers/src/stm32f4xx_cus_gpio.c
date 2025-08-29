#include "stm32f4xx_cus_gpio.h"

void GPIO_INIT(GPIO_Handle_TypeDef *gpioHandle) {
	// Enable clock source
	if (gpioHandle->GPIOX == GPIOA) {
		GPIOA_EN();
	} else if (gpioHandle->GPIOX == GPIOB) {
		GPIOB_EN();
	} else if (gpioHandle->GPIOX == GPIOC) {
		GPIOC_EN();
	} else if (gpioHandle->GPIOX == GPIOD) {
		GPIOD_EN();
	} else if (gpioHandle->GPIOX == GPIOE) {
		GPIOE_EN();
	} else if (gpioHandle->GPIOX == GPIOF) {
		GPIOF_EN();
	} else if (gpioHandle->GPIOX == GPIOG) {
		GPIOG_EN();
	} else if (gpioHandle->GPIOX == GPIOH) {
		GPIOH_EN();
	} else if (gpioHandle->GPIOX == GPIOI) {
		GPIOI_EN();
	}

	// Reset mode
	gpioHandle->GPIOX->MODER &= ~(GPIO_BIT_11_Mask
			<< (Shift_2_pos * gpioHandle->pin_number));
	// Pull-up or Pull-down
	gpioHandle->GPIOX->PUPDR &= ~(GPIO_BIT_11_Mask
			<< (Shift_2_pos * gpioHandle->pin_number));

	gpioHandle->GPIOX->PUPDR |= (gpioHandle->pull_up_pull_down
			<< (Shift_2_pos * gpioHandle->pin_number));
//	// Handle input mode
//	if (gpioHandle->mode == GPIO_MODE_INPUT) {
//
//	}
// Handle output mode
	if (gpioHandle->mode == GPIO_MODE_OUTPUT) {
		// Set output mode
		gpioHandle->GPIOX->MODER |= (gpioHandle->mode
				<< (Shift_2_pos * gpioHandle->pin_number));

		// Set output type
		gpioHandle->GPIOX->OTYPER &= ~(GPIO_BIT_11_Mask
				<< (gpioHandle->pin_number));
		gpioHandle->GPIOX->OTYPER |= (gpioHandle->output_type)
				<< (gpioHandle->pin_number);

		// Set output speed
		gpioHandle->GPIOX->OSPEEDR &= ~(GPIO_BIT_11_Mask
				<< (Shift_2_pos * gpioHandle->pin_number));
		gpioHandle->GPIOX->OSPEEDR |= (gpioHandle->output_speed)
				<< (Shift_2_pos * gpioHandle->pin_number);
	} else if (gpioHandle->mode == GPIO_MODE_AF) {
		uint8_t ALT_low_high = (gpioHandle->pin_number / Divide_ALT_Function);
		uint8_t ALT_bit = (gpioHandle->pin_number % Divide_ALT_Function);
		gpioHandle->GPIOX->AFR[ALT_low_high] &= ~(GPIO_BIT_11_Mask
				<< gpioHandle->pin_number);
		gpioHandle->GPIOX->AFR[ALT_low_high] |=
				(gpioHandle->alternate_function_select << Shift_4_pos * ALT_bit);

	}
}

uint8_t GPIO_INPUT(GPIO_TypeDef *gpiox, uint8_t gpio_pins) {
	uint8_t val = ((gpiox->IDR >> gpio_pins) & GPIO_BIT_1_Mask);
	return val;
}

void GPIO_TOGGLE(GPIO_TypeDef *gpiox, uint8_t gpio_pins) {
	gpiox->ODR ^= (HIGH << gpio_pins);
}

void GPIO_OUTPUT(GPIO_TypeDef *gpiox, uint8_t gpio_pins, uint8_t val) {
	if (val == HIGH) {
		gpiox->BSRR |= (HIGH << gpio_pins);
	} else {
		gpiox->BSRR |= (HIGH << Shift_16_pos + gpio_pins);
	}
}

