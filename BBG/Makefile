	common_sources= control_node.c
	
	common_includes =	

	common_flags =	-lrt \
			-pthread \
			-lm

	common_objects =

	CC=gcc
	flags =	$(common_flags)
	sources = $(common_sources)
	includes = $(common_includes)
	executables:= $(sources:.elf=.elf)
	objects:= $(sources:.c=.o)
	deps:= $(sources:.c=.d)	

	ifeq ($(PLATFORM),ARM)
		CC =arm-cortex_a8-linux-gnueabihf-gcc
	endif

	ifeq ($(PLATFORM),QEMU)
		CC=arm-unknown-linux-gnueabi-gcc
	endif

	ifeq ($(PLATFORM),BBG)
		CC =~/buildroot/output/host/usr/bin/arm-linux-gcc
	endif

.PHONY:	compile-all \
	clean \
	build \
	upload

build:	control_node.elf

%.elf:%.o
	$(CC)	-o	$@	$<	$(flags)

%.i:%.c		$(includes)
	$(CC)	-E	$<	-o	$@	$(flags)	

%.asm:%.c	$(includes)
	$(CC)	-S	$<	-o	$@	$(flags)	

%.o:%.c		$(includes)
	$(CC)	-c	$<	-o	$@	$(flags)	

%.o:%.s		$(includes)
	$(CC)	-c	$<	-o	$@	$(flags)	
	
%.d:%.c		$(includes)
	$(CC)	-M	$<	-o	$@	$(flags)		

%.d:%.s		$(includes)
	$(CC)	-M	$<	-o	$@	$(flags)	

compile-all:	$(objects) 

clean :
	rm	-f	*.o	*.s	*.i	*.o	*.d	*.elf *.swp

upload:
	make	clean
	sudo	make	PLATFORM=BBG
	sudo	scp	project.elf	led_init.sh	root@$(ip):/
