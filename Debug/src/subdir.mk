################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BlinkLed.c \
../src/Timer.c \
../src/_write.c \
../src/bluetooth.c \
../src/dma.c \
../src/stm32f10x_it.c \
../src/usart.c 

CPP_SRCS += \
../src/CLRCppTest.cpp \
../src/CLRMPU9250.cpp \
../src/IFVRCtrlerSys.cpp \
../src/main.cpp 

OBJS += \
./src/BlinkLed.o \
./src/CLRCppTest.o \
./src/CLRMPU9250.o \
./src/IFVRCtrlerSys.o \
./src/Timer.o \
./src/_write.o \
./src/bluetooth.o \
./src/dma.o \
./src/main.o \
./src/stm32f10x_it.o \
./src/usart.o 

C_DEPS += \
./src/BlinkLed.d \
./src/Timer.d \
./src/_write.d \
./src/bluetooth.d \
./src/dma.d \
./src/stm32f10x_it.d \
./src/usart.d 

CPP_DEPS += \
./src/CLRCppTest.d \
./src/CLRMPU9250.d \
./src/IFVRCtrlerSys.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/include -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include/cmsis -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include/stm32f1-stdperiph -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/include -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include/cmsis -I/home/clover/workspace/EclipseARMCHome/CLRHelloTimer/system/include/stm32f1-stdperiph -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


