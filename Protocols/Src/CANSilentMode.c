#include "stm32f4xx_cus.h"
#include "stm32f4xx_cus_gpio.h"
#include "stm32f4xx_cus_can.h"
#include <stdio.h>  // For printf

// Function prototypes
void CAN_GPIO_Init(void);
void Test_Silent_Mode(void);
void delay_ms(uint32_t ms);

int main(void) {
	// Run CAN loopback test
	Test_Silent_Mode();

	while (1) {
		// Main loop - test runs in Test_Loopback_Mode
	}
}

void CAN_GPIO_Init(void) {
	// Configure PD0 (CAN1_RX) and PD1 (CAN1_TX)
	GPIO_Handle_TypeDef canTxPin;
	canTxPin.GPIOX = GPIOD;
	canTxPin.pin_number = GPIO_PIN_1; // CAN1_TX = PD1
	canTxPin.mode = GPIO_MODE_AF; // Alternate function
	canTxPin.alternate_function_select = GPIO_AF9; // AF9 = CAN1
	canTxPin.output_type = GPIO_OUTPUT_TYPE_PP; // Push-pull
	canTxPin.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH; // Very high speed
	canTxPin.pull_up_pull_down = GPIO_PUPD_PU; // Pull-up
	GPIO_INIT(&canTxPin);

	GPIO_Handle_TypeDef canRxPin;
	canRxPin.GPIOX = GPIOD;
	canRxPin.pin_number = GPIO_PIN_0; // CAN1_RX = PD0
	canRxPin.mode = GPIO_MODE_AF; // Alternate function
	canRxPin.alternate_function_select = GPIO_AF9; // AF9 = CAN1
	canRxPin.output_type = GPIO_OUTPUT_TYPE_PP; // Push-pull (not used for input but set anyway)
	canRxPin.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH; // Very high speed
	canRxPin.pull_up_pull_down = GPIO_PUPD_PU; // Pull-up
	GPIO_INIT(&canRxPin);
}

void Test_Silent_Mode(void) {
	// Step 1: Initialize GPIO
	CAN_GPIO_Init();

	// Step 2: Configure CAN handle
	CAN_HandleTypedef canHandle;
	canHandle.CANx = CAN1;
	canHandle.transmitPriority = 0; // Priority by request order
	canHandle.receiveFIFOLockedMode = 0; // Overwrite mode (not locked)
	canHandle.autoReTransmit = 1; // Disable auto retransmission
	canHandle.autoBusOff = 1; // Enable auto bus-off management
	canHandle.bitrate = 1000000; // 1000 kbps
	canHandle.testModeSelected = CAN_SILENT_MODE; // Silent mode

	// Step 3: Initialize CAN peripheral
	CAN_INIT(&canHandle);

	// Step 4: Configure filter to accept all messages
	CAN_FilterHandleTypeDef filterConfig;
	filterConfig.filterBank = 0;
	filterConfig.filterScale = CAN_FILTER_32_BIT_SCALE;
	filterConfig.filterMaskListMode = CAN_ID_MASK_MODE;
	filterConfig.extd = CAN_STANDARD_IDE;
	filterConfig.fifoAssignment = CAN_FIFO_0;

	// Accept all messages (ID = 0x000, Mask = 0x000)
	filterConfig.CAN_Filter32BitMaskHandleTypeDef.id1 = 0x000;
	filterConfig.CAN_Filter32BitMaskHandleTypeDef.mask1 = 0x000;

	CAN_FILTER_CONFIG(&canHandle, &filterConfig);

	printf("CAN Loopback Test Started\r\n");
	printf("==========================\r\n\r\n");

	while (1) {

		// Small delay to allow transmission to complete
		delay_ms(10);

		// Check for received messages
		uint8_t msgPending = CAN_RX_FREE_LEVEL(&canHandle, CAN_FIFO_0);

		if (msgPending > 0) {
			CAN_RXHandleTypeDef rxMsg;
			uint8_t status = CAN_RECEIVE(&canHandle, &rxMsg, CAN_FIFO_0);

			if (status == CAN_RCV_SUCCESS) {
				// Print received message details
				printf("Message received successfully!\r\n");
				printf("  ID: 0x%03lX\r\n", rxMsg.identifier);
				printf("  Type: %s\r\n",
						rxMsg.extd == CAN_STANDARD_IDE ?
								"Standard" : "Extended");
				printf("  Frame: %s\r\n", rxMsg.rtr ? "Remote" : "Data");
				printf("  DLC: %d\r\n", rxMsg.dataLengthCode);
				printf("  Data: ");

				for (uint8_t i = 0; i < rxMsg.dataLengthCode; i++) {
					printf("0x%02X ", rxMsg.data[i]);
				}
				printf("\r\n");
			} else {
				printf("Failed to receive message\r\n");
			}
		} else {
			printf("No message pending in FIFO\r\n");
		}

		printf("\r\n");

		// Delay between messages (1 second)
		delay_ms(1000);
	}
}

void delay_ms(uint32_t ms) {
	// Rough delay - adjust based on your system clock
	// Assumes ~16 MHz clock, ~4 cycles per loop iteration
	for (uint32_t i = 0; i < ms; i++) {
		for (volatile uint32_t j = 0; j < 4000; j++)
			;
	}
}
