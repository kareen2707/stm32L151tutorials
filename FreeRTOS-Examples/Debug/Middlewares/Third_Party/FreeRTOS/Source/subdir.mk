################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/croutine.o: ../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/croutine.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/event_groups.o: ../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/event_groups.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/list.o: ../Middlewares/Third_Party/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/list.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/queue.o: ../Middlewares/Third_Party/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/queue.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o: ../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/tasks.o: ../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/tasks.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/timers.o: ../Middlewares/Third_Party/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L151xD -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../MEMS/Target -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../MEMS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/hts221/ -I../FATFS/Target -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

