ifeq (y, $(strip $(EXT_OL_EXAMPLE)))
CFLAGS += -DEXT_OL_EXAMPLE
SRCS += $(PROJECT)/src/examples/ext_ol_example/ext_ol_example.c
INCS += $(PROJECT)/src/examples/ext_ol_example/ext_ol_example.h
endif

ifeq (y, $(strip $(INT_OL_THETA_EXAMPLE)))
CFLAGS += -DINT_OL_THETA_EXAMPLE
SRCS += $(PROJECT)/src/examples/int_ol_theta_example/int_ol_theta_example.c
INCS += $(PROJECT)/src/examples/int_ol_theta_example/int_ol_theta_example.h
endif

ifeq (y, $(strip $(INT_OL_DQWT_EXAMPLE)))
CFLAGS += -DINT_OL_DQWT_EXAMPLE
SRCS += $(PROJECT)/src/examples/int_ol_dqwt_example/int_ol_dqwt_example.c
INCS += $(PROJECT)/src/examples/int_ol_dqwt_example/int_ol_dqwt_example.h
endif

ifeq (y, $(strip $(INT_OL_ALL_EXAMPLE)))
CFLAGS += -DINT_OL_ALL_EXAMPLE
SRCS += $(PROJECT)/src/examples/int_ol_all_example/int_ol_all_example.c
INCS += $(PROJECT)/src/examples/int_ol_all_example/int_ol_all_example.h
endif
