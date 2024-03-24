obj-m := tas5805m.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp $(shell pwd)/tas5805m.ko /lib/modules/$(shell uname -r)/kernel/sound/soc/codecs/snd-soc-tas5805m.ko
	sudo depmod -a
