#include <stdint.h>
#include <main.h>

GPIO_Handle_TypeDef gpio_handle;
TIM_HandleTypeDef tim_millis_handle;

void GPIO_USER_INIT(void);
void TIMER_USER_INIT(void);

int main(void) {
	GPIO_USER_INIT();
	TIMER_USER_INIT();

	uint32_t previousMillis = 0;
	const uint32_t interval = 1000;  // 1 second in milliseconds

	while (1) {
		uint32_t currentMillis = millis();

		// Non-blocking blink using millis()
		if (currentMillis - previousMillis >= interval) {
			previousMillis = currentMillis;

			// Toggle LED
			GPIO_TOGGLE(GPIOA, GPIO_PIN_8);
		}

		// Your other code can run here without blocking!
	}
}

void GPIO_USER_INIT(void) {
	gpio_handle.GPIOX = GPIOA;
	gpio_handle.pin_number = GPIO_PIN_8;
	gpio_handle.mode = GPIO_MODE_OUTPUT;
	gpio_handle.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH;
	gpio_handle.output_type = GPIO_OUTPUT_TYPE_PP;
	gpio_handle.pull_up_pull_down = GPIO_PUPD_NONE;
	GPIO_INIT(&gpio_handle);
}

void TIMER_USER_INIT(void) {
	tim_millis_handle.TIMx = TIM2;
	TIM_MILLIS_INIT(&tim_millis_handle);
	TIM_ENABLE(&tim_millis_handle);
}

// Interrupt handler
void TIM2_IRQHandler(void) {
	TIM_Handling(&tim_millis_handle);
}
