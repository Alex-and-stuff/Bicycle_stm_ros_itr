################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CanLogger.c \
../Core/Src/EKF.c \
../Core/Src/LCD.c \
../Core/Src/LMI_balance.c \
../Core/Src/MatrixCalculate.c \
../Core/Src/MxMotor.c \
../Core/Src/OpenIMU.c \
../Core/Src/PDcontrol.c \
../Core/Src/QuadrantAngle.c \
../Core/Src/Tracking.c \
../Core/Src/can.c \
../Core/Src/controller.c \
../Core/Src/dma.c \
../Core/Src/dxl_hal.c \
../Core/Src/dynamixel.c \
../Core/Src/estimator.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/sensor_fusion.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

CPP_SRCS += \
../Core/Src/mainpp.cpp 

C_DEPS += \
./Core/Src/CanLogger.d \
./Core/Src/EKF.d \
./Core/Src/LCD.d \
./Core/Src/LMI_balance.d \
./Core/Src/MatrixCalculate.d \
./Core/Src/MxMotor.d \
./Core/Src/OpenIMU.d \
./Core/Src/PDcontrol.d \
./Core/Src/QuadrantAngle.d \
./Core/Src/Tracking.d \
./Core/Src/can.d \
./Core/Src/controller.d \
./Core/Src/dma.d \
./Core/Src/dxl_hal.d \
./Core/Src/dynamixel.d \
./Core/Src/estimator.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/sensor_fusion.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/CanLogger.o \
./Core/Src/EKF.o \
./Core/Src/LCD.o \
./Core/Src/LMI_balance.o \
./Core/Src/MatrixCalculate.o \
./Core/Src/MxMotor.o \
./Core/Src/OpenIMU.o \
./Core/Src/PDcontrol.o \
./Core/Src/QuadrantAngle.o \
./Core/Src/Tracking.o \
./Core/Src/can.o \
./Core/Src/controller.o \
./Core/Src/dma.o \
./Core/Src/dxl_hal.o \
./Core/Src/dynamixel.o \
./Core/Src/estimator.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/mainpp.o \
./Core/Src/sensor_fusion.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

CPP_DEPS += \
./Core/Src/mainpp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CanLogger.d ./Core/Src/CanLogger.o ./Core/Src/CanLogger.su ./Core/Src/EKF.d ./Core/Src/EKF.o ./Core/Src/EKF.su ./Core/Src/LCD.d ./Core/Src/LCD.o ./Core/Src/LCD.su ./Core/Src/LMI_balance.d ./Core/Src/LMI_balance.o ./Core/Src/LMI_balance.su ./Core/Src/MatrixCalculate.d ./Core/Src/MatrixCalculate.o ./Core/Src/MatrixCalculate.su ./Core/Src/MxMotor.d ./Core/Src/MxMotor.o ./Core/Src/MxMotor.su ./Core/Src/OpenIMU.d ./Core/Src/OpenIMU.o ./Core/Src/OpenIMU.su ./Core/Src/PDcontrol.d ./Core/Src/PDcontrol.o ./Core/Src/PDcontrol.su ./Core/Src/QuadrantAngle.d ./Core/Src/QuadrantAngle.o ./Core/Src/QuadrantAngle.su ./Core/Src/Tracking.d ./Core/Src/Tracking.o ./Core/Src/Tracking.su ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/controller.d ./Core/Src/controller.o ./Core/Src/controller.su ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/dxl_hal.d ./Core/Src/dxl_hal.o ./Core/Src/dxl_hal.su ./Core/Src/dynamixel.d ./Core/Src/dynamixel.o ./Core/Src/dynamixel.su ./Core/Src/estimator.d ./Core/Src/estimator.o ./Core/Src/estimator.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mainpp.d ./Core/Src/mainpp.o ./Core/Src/mainpp.su ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

