SDCC=sdcc
SDLD=sdld
CFLAGS=-mstm8 --opt-code-size
LDFLAGS=-lstm8

.PHONY: all clean flash

all:
	$(SDCC) -c ../drivers/stm8s_gpio.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../drivers/stm8s_clk.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../drivers/stm8s_tim1.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../drivers/stm8s_uart1.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c printf.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c main.c -I../include -I./ $(CFLAGS) $(LDFLAGS)
	$(SDCC) main.rel printf.rel stm8s_gpio.rel stm8s_clk.rel stm8s_tim1.rel stm8s_uart1.rel $(CFLAGS) $(LDFLAGS)

clean:
	rm -rf *.asm *.sym *.lst *.rel *.ihx *.map *.lk *.rst

flash: main.ihx ../tools/stm8flash/stm8flash
	../tools/stm8flash/stm8flash -c stlinkv2 -p stm8s103?3 -w main.ihx

../tools/stm8flash/stm8flash:
	make -C ../tools/stm8flash
