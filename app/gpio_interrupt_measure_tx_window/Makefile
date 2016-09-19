SDCC=sdcc
SDLD=sdld
INCLUDE=-I../../include/stm8s -I./ -I../../lib
CFLAGS=$(INCLUDE) -mstm8 --opt-code-size
LDFLAGS=-lstm8

.PHONY: all clean flash

all:
	$(SDCC) -c ../../drivers/stm8s/stm8s_gpio.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8s/stm8s_clk.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8s/stm8s_tim1.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8s/stm8s_uart1.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8s/stm8s_exti.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../lib/printf.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c main.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) main.rel printf.rel stm8s_gpio.rel stm8s_clk.rel stm8s_tim1.rel stm8s_uart1.rel stm8s_exti.rel $(CFLAGS) $(LDFLAGS)

clean:
	rm -rf *.asm *.sym *.lst *.rel *.ihx *.map *.lk *.rst

flash: main.ihx ../tools/stm8flash/stm8flash
	../../tools/stm8flash/stm8flash -c stlinkv2 -p stm8s103?3 -w main.ihx

../tools/stm8flash/stm8flash:
	make -C ../../tools/stm8flash
