#ifndef INC_STM32F4XX_CUS_I2C_H_
#define INC_STM32F4XX_CUS_I2C_H_
#include "stm32f4xx_cus.h"

/**
 * @brief I2C Register Structure Definition for STM32F407
 * @note Based on STM32F4xx I2C register map (RM0090)
 */
typedef struct {
	__IO uint32_t CR1; /*!< I2C Control register 1,     Address offset: 0x00 */
	__IO uint32_t CR2; /*!< I2C Control register 2,     Address offset: 0x04 */
	__IO uint32_t OAR1; /*!< I2C Own address register 1, Address offset: 0x08 */
	__IO uint32_t OAR2; /*!< I2C Own address register 2, Address offset: 0x0C */
	__IO uint32_t DR; /*!< I2C Data register,          Address offset: 0x10 */
	__IO uint32_t SR1; /*!< I2C Status register 1,      Address offset: 0x14 */
	__IO uint32_t SR2; /*!< I2C Status register 2,      Address offset: 0x18 */
	__IO uint32_t CCR; /*!< I2C Clock control register, Address offset: 0x1C */
	__IO uint32_t TRISE; /*!< I2C TRISE register,         Address offset: 0x20 */
	__IO uint32_t FLTR; /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;

typedef struct {
	I2C_TypeDef *I2Cx;
	uint8_t mode;
	uint8_t MasterOrSlave;
	uint8_t address_select_bit;
	uint8_t address;
	uint32_t scl_speed;
	uint8_t duty_cycle;
	uint8_t ack_en;
} I2C_Handle_TypeDef;

typedef struct {
	uint8_t *ptx;
	uint8_t *prx;
	uint8_t state;
	uint32_t tx_len;
	uint32_t rx_len;
	uint8_t addr;
} I2C_Handle_IT;

extern I2C_Handle_IT I2C_Handle_it;
extern uint8_t rx_complete;
extern uint8_t tx_complete;

/**
 * @brief I2C Base Address Definitions
 */
#define I2C1_BASE           0x40005400UL
#define I2C2_BASE           0x40005800UL
#define I2C3_BASE           0x40005C00UL

#define I2C1                ((I2C_TypeDef*)I2C1_BASE)
#define I2C2                ((I2C_TypeDef*)I2C2_BASE)
#define I2C3                ((I2C_TypeDef*)I2C3_BASE)

// Mode
#define I2C_Standard_Mode 0x0
#define I2C_Fast_Mode 0x1

// SCL_Speed
#define I2C_SCL_SPEED_100 100000UL
#define I2C_SCL_SPEED_400 400000UL

// Master/Slave
#define I2C_Master_Mode 0x0
#define I2C_Slave_Mode 0x1

// 7-bit/10-bit
#define I2C_7_Bit_Adress 0x0
#define I2C_10_Bit_Adress 0x1

// Duty cycle
#define I2C_Duty_Cycle_2 0x0
#define I2C_Duty_Cycle_16_9 0x1

// Acknowledge enable
#define I2C_ACK_DIS 0x0
#define I2C_ACK_EN 0x1

// Trise Maximum
#define I2C_TRISE_MAX_FAST_MODE 300

// Read/Write bit
#define I2C_WRITE_BIT 0
#define I2C_READ_BIT 1

// Repeated Start
#define I2C_SR_DIS 0
#define I2C_SR_EN 1

// Interrupt State
#define I2C_READY 0
#define I2C_BUSY_TX 1
#define I2C_BUSY_RX 2

// Function prototypes
void I2C_INIT(I2C_Handle_TypeDef *i2c_handle);
void I2C_Master_Write(I2C_Handle_TypeDef *i2c_handle, uint8_t addr,
		uint8_t *data, uint32_t size, uint8_t sr);
void I2C_Master_Read(I2C_Handle_TypeDef *i2c_handle, uint8_t addr,
		uint8_t *data, uint8_t size, uint8_t sr);
void I2C_Address(I2C_Handle_TypeDef *i2c_handle, uint8_t addr, uint8_t rnw);

// Interrupt
uint8_t I2C_Master_Write_IT(I2C_Handle_TypeDef *i2c_handle, uint8_t addr,
		uint8_t *data, uint32_t size, uint8_t sr);
uint8_t I2C_Master_Read_IT(I2C_Handle_TypeDef *i2c_handle, uint8_t addr,
		uint8_t *data, uint8_t size, uint8_t sr);
void I2C_EV_IRQ_Handling(I2C_Handle_TypeDef *i2c_handle);

#endif /* INC_STM32F4XX_CUS_I2C_H_ */
