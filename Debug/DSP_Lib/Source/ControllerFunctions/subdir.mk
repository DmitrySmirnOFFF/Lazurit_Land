################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.c \
../DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.c \
../DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.c \
../DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.c \
../DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.c \
../DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.c \
../DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.c \
../DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.o \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.o \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.o \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.o \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.o \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.o \
./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.o \
./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.d \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.d \
./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.d \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.d \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.d \
./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.d \
./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.d \
./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DSP_Lib/Source/ControllerFunctions/%.o DSP_Lib/Source/ControllerFunctions/%.su DSP_Lib/Source/ControllerFunctions/%.cyclo: ../DSP_Lib/Source/ControllerFunctions/%.c DSP_Lib/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -D__FPU_PRESENT -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/SOFT/Cube_IDE_pro/MAKET_VID_LAND/DSP_Lib/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP_Lib-2f-Source-2f-ControllerFunctions

clean-DSP_Lib-2f-Source-2f-ControllerFunctions:
	-$(RM) ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.su ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.su ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.su ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.su ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.su ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.d ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.o ./DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.su ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.d ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.o ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.su ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.d ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.o ./DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-DSP_Lib-2f-Source-2f-ControllerFunctions

