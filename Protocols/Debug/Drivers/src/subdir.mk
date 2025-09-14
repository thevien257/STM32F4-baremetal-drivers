################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/stm32f4xx_cus.c \
../Drivers/src/stm32f4xx_cus_gpio.c \
../Drivers/src/stm32f4xx_cus_i2c.c \
../Drivers/src/stm32f4xx_cus_spi.c 

OBJS += \
./Drivers/src/stm32f4xx_cus.o \
./Drivers/src/stm32f4xx_cus_gpio.o \
./Drivers/src/stm32f4xx_cus_i2c.o \
./Drivers/src/stm32f4xx_cus_spi.o 

C_DEPS += \
./Drivers/src/stm32f4xx_cus.d \
./Drivers/src/stm32f4xx_cus_gpio.d \
./Drivers/src/stm32f4xx_cus_i2c.d \
./Drivers/src/stm32f4xx_cus_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o Drivers/src/%.su Drivers/src/%.cyclo: ../Drivers/src/%.c Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/ADMIN/Desktop/Protocols/Protocols/Drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-src

clean-Drivers-2f-src:
	-$(RM) ./Drivers/src/stm32f4xx_cus.cyclo ./Drivers/src/stm32f4xx_cus.d ./Drivers/src/stm32f4xx_cus.o ./Drivers/src/stm32f4xx_cus.su ./Drivers/src/stm32f4xx_cus_gpio.cyclo ./Drivers/src/stm32f4xx_cus_gpio.d ./Drivers/src/stm32f4xx_cus_gpio.o ./Drivers/src/stm32f4xx_cus_gpio.su ./Drivers/src/stm32f4xx_cus_i2c.cyclo ./Drivers/src/stm32f4xx_cus_i2c.d ./Drivers/src/stm32f4xx_cus_i2c.o ./Drivers/src/stm32f4xx_cus_i2c.su ./Drivers/src/stm32f4xx_cus_spi.cyclo ./Drivers/src/stm32f4xx_cus_spi.d ./Drivers/src/stm32f4xx_cus_spi.o ./Drivers/src/stm32f4xx_cus_spi.su

.PHONY: clean-Drivers-2f-src

