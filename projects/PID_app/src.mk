include $(PROJECT)/src/platform/$(PLATFORM)/platform_src.mk
include $(PROJECT)/src/examples/examples_src.mk

SRCS += $(PROJECT)/src/platform/$(PLATFORM)/main.c

INCS += $(PROJECT)/src/common/common_data.h
SRCS += $(PROJECT)/src/common/common_data.c

INCS += $(PROJECT)/src/platform/platform_includes.h

INCS += $(PROJECT)/src/platform/$(PLATFORM)/parameters.h
SRCS += $(PROJECT)/src/platform/$(PLATFORM)/parameters.c

INCS += $(INCLUDE)/no_os_delay.h	\
	$(INCLUDE)/no_os_error.h	\
	$(INCLUDE)/no_os_gpio.h		\
	$(INCLUDE)/no_os_print_log.h	\
	$(INCLUDE)/no_os_alloc.h	\
	$(INCLUDE)/no_os_irq.h		\
	$(INCLUDE)/no_os_list.h		\
	$(INCLUDE)/no_os_dma.h		\
	$(INCLUDE)/no_os_uart.h		\
	$(INCLUDE)/no_os_lf256fifo.h	\
	$(INCLUDE)/no_os_util.h		\
	$(INCLUDE)/no_os_units.h	\
	$(INCLUDE)/no_os_spi.h		\
	$(INCLUDE)/no_os_pwm.h		\
	$(INCLUDE)/no_os_mutex.h

SRCS += $(DRIVERS)/api/no_os_gpio.c	\
	$(NO-OS)/util/no_os_lf256fifo.c \
	$(DRIVERS)/api/no_os_irq.c	\
	$(DRIVERS)/api/no_os_uart.c	\
	$(DRIVERS)/api/no_os_spi.c	\
	$(DRIVERS)/api/no_os_dma.c	\
	$(DRIVERS)/api/no_os_pwm.c	\
	$(NO-OS)/util/no_os_list.c	\
	$(NO-OS)/util/no_os_util.c	\
	$(NO-OS)/util/no_os_alloc.c	\
	$(NO-OS)/util/no_os_mutex.c

SRCS += $(PROJECT)/transform/vector_transfs.c
INCS += $(PROJECT)/transform/vector_transfs.h

SRCS += $(PROJECT)/svpwm/svpwm.c
INCS += $(PROJECT)/svpwm/svpwm.h

SRCS += $(PROJECT)/estimators/fp_pid.c
INCS += $(PROJECT)/estimators/fp_pid.h

# SRCS += $(PROJECT)/estimators/im_estimators.c
# INCS += $(PROJECT)/estimators/im_estimators.h

INCS += $(DRIVERS)/motor/tmc4671/tmc4671.h
SRCS += $(DRIVERS)/motor/tmc4671/tmc4671.c

INCS += $(DRIVERS)/motor/tmc6100/tmc6100.h
SRCS += $(DRIVERS)/motor/tmc6100/tmc6100.c
