################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/MPU6050DMP/CLRMPU6050DMP.cpp \
../src/MPU6050DMP/MPU6050.cpp 

OBJS += \
./src/MPU6050DMP/CLRMPU6050DMP.o \
./src/MPU6050DMP/MPU6050.o 

CPP_DEPS += \
./src/MPU6050DMP/CLRMPU6050DMP.d \
./src/MPU6050DMP/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
src/MPU6050DMP/%.o: ../src/MPU6050DMP/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


