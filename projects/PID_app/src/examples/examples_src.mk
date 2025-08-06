ifeq (y,$(strip $(EXT_LOOP_EXAMPLE)))
SRCS += $(PROJECT)/src/examples/ext_loop/ext_loop_example.c
INCS += $(PROJECT)/src/examples/ext_loop/ext_loop_example.h
endif

ifeq (y,$(strip $(EXT_TRANSF_EXAMPLE)))
SRCS += $(PROJECT)/src/examples/ext_transf/ext_transf_example.c
INCS += $(PROJECT)/src/examples/ext_transf/ext_transf_example.h
endif

ifeq (y,$(strip $(EXT_CTRL_EXAMPLE)))
SRCS += $(PROJECT)/src/examples/ext_ctrl/ext_ctrl_example.c
INCS += $(PROJECT)/src/examples/ext_ctrl/ext_ctrl_example.h
endif
