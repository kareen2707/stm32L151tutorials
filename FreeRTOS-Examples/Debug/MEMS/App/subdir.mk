################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MEMS/App/app_x-cube-mems1.c 

OBJS += \
./MEMS/App/app_x-cube-mems1.o 

C_DEPS += \
./MEMS/App/app_x-cube-mems1.d 


# Each subdirectory must supply rules for building sources it contributes
MEMS/App/app_x-cube-mems1.o: ../MEMS/App/app_x-cube-mems1.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MEMS/App/app_x-cube-mems1.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
