################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hts221/hts221.c \
../Drivers/BSP/Components/hts221/hts221_reg.c 

OBJS += \
./Drivers/BSP/Components/hts221/hts221.o \
./Drivers/BSP/Components/hts221/hts221_reg.o 

C_DEPS += \
./Drivers/BSP/Components/hts221/hts221.d \
./Drivers/BSP/Components/hts221/hts221_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hts221/hts221.o: ../Drivers/BSP/Components/hts221/hts221.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hts221/hts221.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/BSP/Components/hts221/hts221_reg.o: ../Drivers/BSP/Components/hts221/hts221_reg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hts221/hts221_reg.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

