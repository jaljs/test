SDCC=sdcc
SDLD=sdld
INCLUDE=-I../../include/stm8l151x -I./ -I../../lib
CFLAGS=$(INCLUDE) -mstm8 --opt-code-size
LDFLAGS=-lstm8

.PHONY: all clean flash

all:
	$(SDCC) -c main.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) main.rel $(CFLAGS) $(LDFLAGS)

clean:
	rm -rf *.asm *.sym *.lst *.rel *.ihx *.map *.lk *.rst

flash: main.ihx ../tools/stm8flash/stm8flash
	../../tools/stm8flash/stm8flash -c stlinkv2 -p stm8l151?6 -w main.ihx

../tools/stm8flash/stm8flash:
	make -C ../../tools/stm8flash
