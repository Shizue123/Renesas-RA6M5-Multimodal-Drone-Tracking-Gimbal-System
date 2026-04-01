################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/control/pid_controller.c \
../src/control/tracking_controller.c 

C_DEPS += \
./src/control/pid_controller.d \
./src/control/tracking_controller.d 

OBJS += \
./src/control/pid_controller.o \
./src/control/tracking_controller.o 

SREC += \
Drone_Tracking_RA6M5.srec 

MAP += \
Drone_Tracking_RA6M5.map 


# Each subdirectory must supply rules for building sources it contributes
src/control/%.o: ../src/control/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-strict-aliasing -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -D_RENESAS_RA_ -D_RA_CORE=CM33 -D_RA_ORDINAL=1 -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra_gen" -I"." -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra_cfg/fsp_cfg/bsp" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra_cfg/fsp_cfg" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/src" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra/fsp/inc" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra/fsp/inc/api" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra/fsp/inc/instances" -I"D:/Renesas-Workspace/Drone_Tracking_RA6M5/ra/arm/CMSIS_6/CMSIS/Core/Include" -std=c99 -Wno-stringop-overflow -Wno-format-truncation --param=min-pagesize=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

