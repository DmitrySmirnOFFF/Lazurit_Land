################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.c \
../DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.c 

OBJS += \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.o \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.o 

C_DEPS += \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.d \
./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DSP_Lib/Source/ComplexMathFunctions/%.o DSP_Lib/Source/ComplexMathFunctions/%.su DSP_Lib/Source/ComplexMathFunctions/%.cyclo: ../DSP_Lib/Source/ComplexMathFunctions/%.c DSP_Lib/Source/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -D__FPU_PRESENT -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Cpro/Lazurit/Lazurit_Land/DSP_Lib/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP_Lib-2f-Source-2f-ComplexMathFunctions

clean-DSP_Lib-2f-Source-2f-ComplexMathFunctions:
	-$(RM) ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_conj_q31.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_q31.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.su ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.cyclo ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.d ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.o ./DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.su

.PHONY: clean-DSP_Lib-2f-Source-2f-ComplexMathFunctions

