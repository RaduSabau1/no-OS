ifeq (y, $(strip $(EXT_LOOP_EXAMPLE)))
CFLAGS += -DEXT_LOOP_EXAMPLE
SRCS += $(PROJECT)/src/examples/ext_loop/ext_loop_example.c
INCS += $(PROJECT)/src/examples/ext_loop/ext_loop_example.h
endif

ifeq (y, $(strip $(INT_LOOP_EXAMPLE)))
CFLAGS += -DINT_LOOP_EXAMPLE
SRCS += $(PROJECT)/src/examples/int_loop_example/int_loop_example.c
INCS += $(PROJECT)/src/examples/int_loop_example/int_loop_example.h
endif

ifeq (y, $(strip $(OPEN_LOOP_EXAMPLE)))
CFLAGS += -DOPEN_LOOP_EXAMPLE
SRCS += $(PROJECT)/src/examples/open_loop/open_loop_example.c
INCS += $(PROJECT)/src/examples/open_loop/open_loop_example.h
endif
