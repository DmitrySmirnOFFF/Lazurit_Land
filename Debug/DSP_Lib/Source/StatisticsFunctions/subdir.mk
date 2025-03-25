################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP_Lib/Source/StatisticsFunctions/arm_max_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_max_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_max_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_max_q7.c \
../DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.c \
../DSP_Lib/Source/StatisticsFunctions/arm_min_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_min_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_min_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_min_q7.c \
../DSP_Lib/Source/StatisticsFunctions/arm_power_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_power_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_power_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_power_q7.c \
../DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_std_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_std_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_std_q31.c \
../DSP_Lib/Source/StatisticsFunctions/arm_var_f32.c \
../DSP_Lib/Source/StatisticsFunctions/arm_var_q15.c \
../DSP_Lib/Source/StatisticsFunctions/arm_var_q31.c 

OBJS += \
./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.o \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.o \
./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.o \
./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.o \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.o \
./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.o \
./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.o \
./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.o 

C_DEPS += \
./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.d \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.d \
./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.d \
./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.d \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.d \
./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.d \
./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.d \
./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DSP_Lib/Source/StatisticsFunctions/%.o DSP_Lib/Source/StatisticsFunctions/%.su DSP_Lib/Source/StatisticsFunctions/%.cyclo: ../DSP_Lib/Source/StatisticsFunctions/%.c DSP_Lib/Source/StatisticsFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -D__FPU_PRESENT -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Cpro/Lazurit/Lazurit_Land/DSP_Lib/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP_Lib-2f-Source-2f-StatisticsFunctions

clean-DSP_Lib-2f-Source-2f-StatisticsFunctions:
	-$(RM) ./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_max_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_max_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_max_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.d ./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.o ./DSP_Lib/Source/StatisticsFunctions/arm_max_q7.su ./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_mean_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.d ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.o ./DSP_Lib/Source/StatisticsFunctions/arm_mean_q7.su ./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_min_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_min_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_min_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.d ./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.o ./DSP_Lib/Source/StatisticsFunctions/arm_min_q7.su ./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_power_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_power_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_power_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.d ./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.o ./DSP_Lib/Source/StatisticsFunctions/arm_power_q7.su ./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_rms_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_rms_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_std_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_std_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_std_q31.su ./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.d ./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.o ./DSP_Lib/Source/StatisticsFunctions/arm_var_f32.su ./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.d ./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.o ./DSP_Lib/Source/StatisticsFunctions/arm_var_q15.su ./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.cyclo ./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.d ./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.o ./DSP_Lib/Source/StatisticsFunctions/arm_var_q31.su

.PHONY: clean-DSP_Lib-2f-Source-2f-StatisticsFunctions

