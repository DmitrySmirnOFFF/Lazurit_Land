################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP_Lib/Source/CommonTables/arm_common_tables.c \
../DSP_Lib/Source/CommonTables/arm_const_structs.c 

OBJS += \
./DSP_Lib/Source/CommonTables/arm_common_tables.o \
./DSP_Lib/Source/CommonTables/arm_const_structs.o 

C_DEPS += \
./DSP_Lib/Source/CommonTables/arm_common_tables.d \
./DSP_Lib/Source/CommonTables/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
DSP_Lib/Source/CommonTables/%.o DSP_Lib/Source/CommonTables/%.su DSP_Lib/Source/CommonTables/%.cyclo: ../DSP_Lib/Source/CommonTables/%.c DSP_Lib/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -D__FPU_PRESENT -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Users/anton/Documents/STM32_prj/Lazurit_Land/DSP_Lib/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP_Lib-2f-Source-2f-CommonTables

clean-DSP_Lib-2f-Source-2f-CommonTables:
	-$(RM) ./DSP_Lib/Source/CommonTables/arm_common_tables.cyclo ./DSP_Lib/Source/CommonTables/arm_common_tables.d ./DSP_Lib/Source/CommonTables/arm_common_tables.o ./DSP_Lib/Source/CommonTables/arm_common_tables.su ./DSP_Lib/Source/CommonTables/arm_const_structs.cyclo ./DSP_Lib/Source/CommonTables/arm_const_structs.d ./DSP_Lib/Source/CommonTables/arm_const_structs.o ./DSP_Lib/Source/CommonTables/arm_const_structs.su

.PHONY: clean-DSP_Lib-2f-Source-2f-CommonTables

