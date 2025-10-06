#ifndef INC_STM32F4XX_CUS_SCB_H_
#define INC_STM32F4XX_CUS_SCB_H_

/* STM32F4 System Control Block (SCB) Definitions */
#include <stdint.h>

/* SCB Base Address */
#define SCB_BASE    0xE000ED00UL

/* SCB Register Structure */
typedef struct {
	volatile uint32_t CPUID;   // Offset: 0x000 (R/ )  CPUID Base Register
	volatile uint32_t ICSR; // Offset: 0x004 (R/W)  Interrupt Control and State Register
	volatile uint32_t VTOR; // Offset: 0x008 (R/W)  Vector Table Offset Register
	volatile uint32_t AIRCR; // Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
	volatile uint32_t SCR;     // Offset: 0x010 (R/W)  System Control Register
	volatile uint32_t CCR; // Offset: 0x014 (R/W)  Configuration Control Register
	volatile uint8_t SHP[12]; // Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
	volatile uint32_t SHCSR; // Offset: 0x024 (R/W)  System Handler Control and State Register
	volatile uint32_t CFSR; // Offset: 0x028 (R/W)  Configurable Fault Status Register
	volatile uint32_t HFSR;    // Offset: 0x02C (R/W)  HardFault Status Register
	volatile uint32_t DFSR;  // Offset: 0x030 (R/W)  Debug Fault Status Register
	volatile uint32_t MMFAR; // Offset: 0x034 (R/W)  MemManage Fault Address Register
	volatile uint32_t BFAR;    // Offset: 0x038 (R/W)  BusFault Address Register
	volatile uint32_t AFSR; // Offset: 0x03C (R/W)  Auxiliary Fault Status Register
	volatile uint32_t PFR[2]; // Offset: 0x040 (R/ )  Processor Feature Register
	volatile uint32_t DFR;     // Offset: 0x048 (R/ )  Debug Feature Register
	volatile uint32_t ADR;    // Offset: 0x04C (R/ )  Auxiliary Feature Register
	volatile uint32_t MMFR[4]; // Offset: 0x050 (R/ )  Memory Model Feature Register
	volatile uint32_t ISAR[5]; // Offset: 0x060 (R/ )  Instruction Set Attributes Register
	uint32_t RESERVED0[5];
	volatile uint32_t CPACR; // Offset: 0x088 (R/W)  Coprocessor Access Control Register
} SCB_Type;

/* SCB Peripheral Instance */
#define SCB    ((SCB_Type *) SCB_BASE)

/* CPACR Register Bits */
#define SCB_CPACR_CP10_Pos          20U
#define SCB_CPACR_CP10_Msk          (3UL << SCB_CPACR_CP10_Pos)
#define SCB_CPACR_CP11_Pos          22U
#define SCB_CPACR_CP11_Msk          (3UL << SCB_CPACR_CP11_Pos)

/* Coprocessor Access Privileges */
#define SCB_CPACR_DENIED            0x0  // Access denied
#define SCB_CPACR_PRIVILEGED        0x1  // Privileged access only
#define SCB_CPACR_RESERVED          0x2  // Reserved
#define SCB_CPACR_FULL_ACCESS       0x3  // Full access

/* AIRCR Register Key */
#define SCB_AIRCR_VECTKEY_Pos       16U
#define SCB_AIRCR_VECTKEY_Msk       (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)
#define SCB_AIRCR_VECTKEYSTAT_Msk   SCB_AIRCR_VECTKEY_Msk
#define SCB_AIRCR_VECTKEY           0x05FA0000UL  // Key for writing to AIRCR

/* SCR Register Bits */
#define SCB_SCR_SLEEPONEXIT_Pos     1U
#define SCB_SCR_SLEEPONEXIT_Msk     (1UL << SCB_SCR_SLEEPONEXIT_Pos)
#define SCB_SCR_SLEEPDEEP_Pos       2U
#define SCB_SCR_SLEEPDEEP_Msk       (1UL << SCB_SCR_SLEEPDEEP_Pos)
#define SCB_SCR_SEVONPEND_Pos       4U
#define SCB_SCR_SEVONPEND_Msk       (1UL << SCB_SCR_SEVONPEND_Pos)

/* CCR Register Bits */
#define SCB_CCR_NONBASETHRDENA_Pos  0U
#define SCB_CCR_NONBASETHRDENA_Msk  (1UL << SCB_CCR_NONBASETHRDENA_Pos)
#define SCB_CCR_USERSETMPEND_Pos    1U
#define SCB_CCR_USERSETMPEND_Msk    (1UL << SCB_CCR_USERSETMPEND_Pos)
#define SCB_CCR_UNALIGN_TRP_Pos     3U
#define SCB_CCR_UNALIGN_TRP_Msk     (1UL << SCB_CCR_UNALIGN_TRP_Pos)
#define SCB_CCR_DIV_0_TRP_Pos       4U
#define SCB_CCR_DIV_0_TRP_Msk       (1UL << SCB_CCR_DIV_0_TRP_Pos)
#define SCB_CCR_BFHFNMIGN_Pos       8U
#define SCB_CCR_BFHFNMIGN_Msk       (1UL << SCB_CCR_BFHFNMIGN_Pos)
#define SCB_CCR_STKALIGN_Pos        9U
#define SCB_CCR_STKALIGN_Msk        (1UL << SCB_CCR_STKALIGN_Pos)

/* Helper Macros */

/* Enable FPU (Floating Point Unit) */
#define SCB_ENABLE_FPU() \
    do { \
        SCB->CPACR |= ((3UL << SCB_CPACR_CP10_Pos) | (3UL << SCB_CPACR_CP11_Pos)); \
        __asm volatile ("DSB"); \
        __asm volatile ("ISB"); \
    } while(0)

/* System Reset */
#define SCB_SYSTEM_RESET() \
    do { \
        SCB->AIRCR = (SCB_AIRCR_VECTKEY | (SCB->AIRCR & 0x0000FFFF) | (1UL << 2)); \
        __asm volatile ("DSB"); \
        while(1); \
    } while(0)

#endif /* INC_STM32F4XX_CUS_SCB_H_ */
