################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GlobalFunc.c \
../Core/Src/homing.c \
../Core/Src/interpretaComando.c \
../Core/Src/inverseJacobian.c \
../Core/Src/kinematic.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/statesMachine.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/trajectory.c 

OBJS += \
./Core/Src/GlobalFunc.o \
./Core/Src/homing.o \
./Core/Src/interpretaComando.o \
./Core/Src/inverseJacobian.o \
./Core/Src/kinematic.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/statesMachine.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/trajectory.o 

C_DEPS += \
./Core/Src/GlobalFunc.d \
./Core/Src/homing.d \
./Core/Src/interpretaComando.d \
./Core/Src/inverseJacobian.d \
./Core/Src/kinematic.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/statesMachine.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/GlobalFunc.d ./Core/Src/GlobalFunc.o ./Core/Src/GlobalFunc.su ./Core/Src/homing.d ./Core/Src/homing.o ./Core/Src/homing.su ./Core/Src/interpretaComando.d ./Core/Src/interpretaComando.o ./Core/Src/interpretaComando.su ./Core/Src/inverseJacobian.d ./Core/Src/inverseJacobian.o ./Core/Src/inverseJacobian.su ./Core/Src/kinematic.d ./Core/Src/kinematic.o ./Core/Src/kinematic.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/statesMachine.d ./Core/Src/statesMachine.o ./Core/Src/statesMachine.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/trajectory.d ./Core/Src/trajectory.o ./Core/Src/trajectory.su

.PHONY: clean-Core-2f-Src

