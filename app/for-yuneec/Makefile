SDCC=sdcc
SDLD=sdld
INCLUDE=-I../../include/stm8l151x -I./ -I../../lib
CFLAGS=$(INCLUDE) -mstm8 --opt-code-size
LDFLAGS=-lstm8

.PHONY: all clean flash

all: gen_mcu_version
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_gpio.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_clk.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_tim1.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_tim2.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_tim3.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_tim4.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_usart.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_exti.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_itc.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_dac.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c ../../drivers/stm8l151x/stm8l15x_syscfg.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) -c main.c $(CFLAGS) $(LDFLAGS)
	$(SDCC) main.rel stm8l15x_gpio.rel stm8l15x_clk.rel stm8l15x_tim1.rel stm8l15x_tim2.rel stm8l15x_tim3.rel stm8l15x_tim4.rel stm8l15x_usart.rel stm8l15x_exti.rel stm8l15x_itc.rel stm8l15x_dac.rel stm8l15x_syscfg.rel $(CFLAGS) $(LDFLAGS)
	@cp main.ihx mcu-$$(git rev-list HEAD --count)-$$(git show --pretty=format:%h HEAD | head -1)-$$(git log -n1 --date=short --pretty=%cd HEAD).ihx
	@echo "Generate MCU FW => mcu-$$(git rev-list HEAD --count)-$$(git show --pretty=format:%h HEAD | head -1)-$$(git log -n1 --date=short --pretty=%cd HEAD).ihx"

gen_mcu_version:
	@echo Generate MCU Version HASH=$$(git show --pretty=format:%h HEAD | head -1)
	@sed -e "s/<mcu_version_hash>/$$(git show --pretty=format:%h HEAD | head -1)/g" \
		< version.h.in > version.h.tmp
	@if diff -q version.h version.h.tmp > /dev/null 2>&1; \
	then \
		rm version.h.tmp* ;\
	else \
		echo "version.h.in => verson.h"; \
		mv version.h.tmp version.h; \
	fi

clean:
	rm -rf *.asm *.sym *.lst *.rel *.ihx *.map *.lk *.rst version.h

flash: main.ihx ../tools/stm8flash/stm8flash
	../../tools/stm8flash/stm8flash -c stlinkv2 -p stm8l151?6 -w main.ihx

../tools/stm8flash/stm8flash:
	make -C ../../tools/stm8flash
