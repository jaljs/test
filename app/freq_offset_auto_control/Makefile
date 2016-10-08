SDCC=sdcc
SDLD=sdld
INCLUDE=-I../../include/stm8l151x -I./ -I../../lib
CFLAGS=$(INCLUDE) -mstm8 --opt-code-size
LDFLAGS=-lstm8

.PHONY: all clean flash

all:
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_gpio.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_clk.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_tim1.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_usart.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_syscfg.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_dac.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c main.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) main.rel stm8l15x_gpio.rel stm8l15x_clk.rel stm8l15x_tim1.rel stm8l15x_usart.rel stm8l15x_syscfg.rel stm8l15x_dac.rel $(CFLAGS) $(LDFLAGS)

clean:
	rm -rf *.asm *.sym *.lst *.rel *.ihx *.map *.lk *.rst

flash: main.ihx ../tools/stm8flash/stm8flash
	../../tools/stm8flash/stm8flash -c stlinkv2 -p stm8l151?6 -w main.ihx

../tools/stm8flash/stm8flash:
	make -C ../../tools/stm8flash
