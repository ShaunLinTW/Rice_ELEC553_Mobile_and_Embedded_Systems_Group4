PWD=$(shell pwd)
KERNEL_BUILD=/lib/modules/$(shell uname -r)/build

obj-m+=group4_device.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
