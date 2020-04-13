# For building for the current running version of Linux
TARGET		?= $(shell uname -r)
HOME=$(shell pwd)
MDIR	?= /lib/modules/$(TARGET)
KDIR	?= $(MDIR)/build

#SYSTEM_MAP     := $(KERNEL_BUILD)/System.map
ifneq ("","$(wildcard /boot/System.map-$(TARGET))")
SYSTEM_MAP      := /boot/System.map-$(TARGET)
else
# Arch
SYSTEM_MAP      := /proc/kallsyms
endif

DRIVER := drivetemp

# Directory below /lib/modules/$(TARGET)/kernel into which to install
# the module:
MOD_SUBDIR = drivers/hwmon

obj-m	:= $(DRIVER).o

MAKEFLAGS += --no-print-directory

.PHONY: all install modules modules_install clean

all: modules

# Targets for running make directly in the external module directory:

modules clean:
	@$(MAKE) -C $(KDIR) M=$(CURDIR) $@ EXTRA_CFLAGS=-g

install: modules_install

modules_install:
	cp $(DRIVER).ko $(KERNEL_MODULES)/kernel/$(MOD_SUBDIR)
	depmod -a -F $(SYSTEM_MAP) $(TARGET)
