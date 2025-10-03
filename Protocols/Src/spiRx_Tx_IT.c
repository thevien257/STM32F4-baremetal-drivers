#include "stm32f4xx_cus.h"
#include <string.h>
void SPI_USER_INIT();
void GPIO_USER_INIT();

SPI_HandleTypedef SPI_Handle;
GPIO_Handle_TypeDef GPIO_Handle;
GPIO_Handle_TypeDef spiIntPin;
char user_data[] = "Hello world\n";
/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte = 'N';

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;
uint32_t i = 0;
uint8_t dummy = 0xFF;

int main() {
	GPIO_USER_INIT();
	SPI_USER_INIT();
	IRQ_Config(SPI2_IRQ36, HIGH);
	IRQ_Config(IRQ23_EXTI9_5, HIGH);
	// strlen: excluding the null terminator itself (\0);
	// sizeof: including the null terminator (\0) and any unused space
	printf("Application running\n");
	while (1) {

		while (!dataAvailable)
			; //wait till data available interrupt from transmitter device(slave)

		IRQ_Config(IRQ23_EXTI9_5, LOW);

		SPI_PERIPHERAL_ENABLE(&SPI_Handle, HIGH);

		memset(RcvBuff, 0, MAX_LEN);  // Clear entire buffer
		i = 0;  // Reset index
		ReadByte = 'N';
		while (ReadByte != '\0' && (i < MAX_LEN)) {

			while (SPI_SendReceive_FullDuplex_IT(&SPI_Handle, &dummy, &ReadByte,
					1) == SPI_BUSY_TX_RX_IT)
				;
			RcvBuff[i++] = ReadByte;
			if (ReadByte == '\0' || (i == MAX_LEN)) {
				RcvBuff[i - 1] = '\0';
				i = 0;
				break;
			}
		}

		while (SPI_GetFlagStatus(&SPI_Handle, Shift_7_pos))
			;

		SPI_PERIPHERAL_ENABLE(&SPI_Handle, LOW);

		printf("Rcvd data = %s\n", RcvBuff);

		dataAvailable = 0;

		IRQ_Config(IRQ23_EXTI9_5, HIGH);
	}
	return 0;
}

void GPIO_USER_INIT(void) {
	// PA0 - USER BUTTON
	GPIO_Handle.GPIOX = GPIOA;
	GPIO_Handle.mode = GPIO_MODE_INPUT;
	GPIO_Handle.pin_number = GPIO_PIN_0;
	GPIO_Handle.pull_up_pull_down = GPIO_PUPD_NONE;
	GPIO_INIT(&GPIO_Handle);

	// PB12 - SPI2_NSS
	GPIO_Handle.GPIOX = GPIOB;
	GPIO_Handle.mode = GPIO_MODE_AF;
	GPIO_Handle.alternate_function_select = GPIO_AF5;
	GPIO_Handle.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH;
	GPIO_Handle.output_type = GPIO_OUTPUT_TYPE_PP;
	GPIO_Handle.pull_up_pull_down = GPIO_PUPD_NONE;
	GPIO_Handle.pin_number = GPIO_PIN_12;
	GPIO_INIT(&GPIO_Handle);

	// PB13 - SPI2_SCLK
	GPIO_Handle.pin_number = GPIO_PIN_13;
	GPIO_INIT(&GPIO_Handle);

	// PB14 - SPI2_MISO
	GPIO_Handle.pin_number = GPIO_PIN_14;
	GPIO_INIT(&GPIO_Handle);

	// PB15 - SPI2_MOSI
	GPIO_Handle.pin_number = GPIO_PIN_15;
	GPIO_INIT(&GPIO_Handle);

	// this is interrupt GPIO
	spiIntPin.GPIOX = GPIOD;
	spiIntPin.pin_number = GPIO_PIN_6;
	spiIntPin.mode = GPIO_MODE_INTERRUPT_FALLING;
	spiIntPin.exti_select = EXTI_PORT_PD;
	spiIntPin.pull_up_pull_down = GPIO_PUPD_PU;
	GPIO_INIT(&spiIntPin);
}

void SPI_USER_INIT() {
	SPI_Handle.SPIx = SPI2;
	SPI_Handle.spi_clock_phase = SPI_DATA_CAPTURED_FIRST_CLOCK;
	SPI_Handle.spi_clock_polarity = SPI_IDLE_LOW;
	SPI_Handle.spi_data_direction = SPI_FULL_DUPLEX_MODE;
	SPI_Handle.spi_frame_format = SPI_8_BIT_FRAME_FORMAT;
	SPI_Handle.spi_master_slave = SPI_MASTER_MODE;
	SPI_Handle.spi_sclk_prescaler = SPI_PRES_32;
	SPI_Handle.spi_ssm = SPI_SOFTWARE_SLAVE_DIS;
	SPI_INIT(&SPI_Handle);
}

void SPI2_IRQHandler() {
	SPI_TxRx_HandlingIT(&SPI_Handle);
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void) {
	GPIO_IRQHandling(GPIO_PIN_6);
	dataAvailable = 1;
}
