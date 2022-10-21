##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.16.0] date: [Tue Oct 18 13:16:17 IDT 2022]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = STM32_MCSDK


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/motorcontrol.c \
Src/mc_api.c \
Src/mc_config.c \
Src/mcp_config.c \
Src/mc_tasks.c \
Src/pwm_curr_fdbk.c \
Src/regular_conversion_manager.c \
Src/mc_math.c \
Src/mc_interface.c \
Src/stm32g4xx_mc_it.c \
Src/mc_parameters.c \
Src/register_interface.c \
Src/mc_perf.c \
Src/usart_aspep_driver.c \
Src/mc_configuration_registers.c \
Src/aspep.c \
Src/stm32g4xx_it.c \
Src/stm32g4xx_hal_msp.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_cordic.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_opamp.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c \
Src/system_stm32g4xx.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/feed_forward_ctrl.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pwm_common.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_torq_ctrl.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_cordic_speed_pos_fdbk.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/mcp.c \
MCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32g431xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DARM_MATH_CM4 \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32G431xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IMCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/Any/Inc \
-IMCSDK_v6.0.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32G431CBUx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
