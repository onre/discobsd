BOARD?=			teensy35
KERNDIR=	${TOPSRC}/sys/mk64/${BOARD}
KERN=		unix

CONFSCRIPT!=if [ x"${BOARD}" = x"teensy35" ] ; then \
		echo "board/teensy35.cfg" ; \
	else \
		echo "does/not/exist" ; \
	fi

help:
	@echo yes, you need it

ocd: kernel
	sudo openocd -f "${CONFSCRIPT}"

gdb-ocd: kernel
	arm-none-eabi-gdb -q \
		-ex 'cd ${KERNDIR}' \
		-ex 'target extended-remote :3333' \
		-ex 'mon reset halt' \
		-ex 'load' \
		${KERNDIR}/${KERN}

.PHONY: help ocd gdb-ocd
