/*
 * UART driver for Kinesis K / Teensy
 *
 * Copyright (c) 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/user.h>
#include <sys/ioctl.h>
#include <sys/tty.h>
#include <sys/systm.h>
#include <sys/kconfig.h>

#include <machine/uart.h>
#include <machine/kinetis.h>

#define CONCAT(x, y) x##y
#define BBAUD(x) CONCAT(B, x)

#ifndef UART_BAUD
#        define UART_BAUD 115200
#endif

struct uart_inst {
    KINETISK_UART_t *inst;
    u_char rx_pin_num;
    u_char tx_pin_num;
    volatile u_long *regbase; /* 00 pdor - can be 20, 40, 80, ...
			       * 04 psor
			       * 08 pcor
			       * 0C ptor
			       * 10 pdir
			       * 14 pddr
			       */
    volatile u_char rx_buffer[RX_BUFFER_SIZE];
    volatile u_char tx_buffer[TX_BUFFER_SIZE];
    volatile u_char rx_buffer_head;
    volatile u_char rx_buffer_tail;
    volatile u_char tx_buffer_head;
    volatile u_char tx_buffer_tail;
};

static struct uart_inst uart[NUART] = {
#ifdef TEENSY35
    {&(*(KINETISK_UART_t *) UART0_ADDR), 16, 17, &GPIOB_PDOR},
#if 0
    {&(*(KINETISK_UART_t *) UART1_ADDR)},
    {&(*(KINETISK_UART_t *) UART2_ADDR)},
    {&(*(KINETISK_UART_t *) UART3_ADDR)},
    {&(*(KINETISK_UART_t *) UART4_ADDR)},
    {&(*(KINETISK_UART_t *) UART5_ADDR)}
#    else
    {},
    {},
    {},
    {},
    {}    
#    endif
    
#endif
};

struct tty uartttys[NUART];

/*
 * as NUART is at most less than 8,
 * we can get away with this. 
 */
static volatile uint8_t transmitting;

void uart0_status_isr(void) {
    uartintr(makedev(UART_MAJOR, 0));
}
void uart0_error_isr(void) {
    uartintr(makedev(UART_MAJOR, 0));
}

void uartinit(int unit) {
    register KINETISK_UART_t *inst;
    int divisor;

    if (unit != 0) {
        printf("uart: unit != 0 not supported yet\n");
        return;
    }

    inst = uart[unit].inst;
    
    switch (unit) {
    case 0:
    case 1:
        if (unit == 0) {
            /* TODO: a fancy abstraction for these. */
            PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
                          PORT_PCR_MUX(3);
            PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
	    uart[unit].rx_pin_num = 0;
	    uart[unit].tx_pin_num = 1;
        } else {
            PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
                          PORT_PCR_MUX(3);
            PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
	    uart[unit].rx_pin_num = 0;
	    uart[unit].tx_pin_num = 1;
        }

        divisor = BAUD2DIV(115200);
	
        /* fixed 115200 for now */
	if (divisor < 32) divisor = 32;
	UART0_BDH = (divisor >> 13) & 0x1F;
	UART0_BDL = (divisor >> 5) & 0xFF;
	UART0_C4 = divisor & 0x1F;
	
	inst->C1 = UART_C1_ILT;
	inst->TWFIFO = 2; // tx watermark, causes S1_TDRE to set
	inst->RWFIFO = 4; // rx watermark, causes S1_RDRF to set
	inst->PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
	inst->C2 = C2_TX_INACTIVE;
	arm_set_irq_prio(IRQ_UART0_STATUS, SPL_TTY);
	break;
    default:
	printf("uart: wtf\n");
	return;
    }
}

int uartopen(dev_t dev, int flag, int mode) {
    register struct uart_inst *uip;
    register struct tty *tp;
    register int unit = minor(dev);
    u_char c;

    if (unit < 0 || unit >= NUART)
	return (ENXIO);

    tp = &uartttys[unit];
    if (!tp->t_addr)
	return (ENXIO);

    uip         = (struct uart_inst *) tp->t_addr;
    tp->t_oproc = uartstart;
    if ((tp->t_state & TS_ISOPEN) == 0) {
	if (tp->t_ispeed == 0) {
	    tp->t_ispeed = BBAUD(UART_BAUD);
	    tp->t_ospeed = BBAUD(UART_BAUD);
	}
	ttychars(tp);
	tp->t_state = TS_ISOPEN | TS_CARR_ON;
	tp->t_flags = ECHO | XTABS | CRMOD | CRTBS | CRTERA |
	    CTLECH | CRTKIL;
    }
    if ((tp->t_state & TS_XCLUDE) && u.u_uid != 0)
	return (EBUSY);

    /* C1, S2, C3 are fields in the UART configuration register.
     * this isn't as bad as it looks.
     */
    c = uip->inst->C1;
    c = (c & ~0x13) | (SERIAL_8N1 & 0x03);
    uip->inst->C1 = c;
    c = uip->inst->S2 & ~0x10;
    uip->inst->S2 = c;
    c = uip->inst->C3 & ~0x10;
    uip->inst->C3 = c;

    /* deep breath - hope for the best - activate isr
     */
    NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
    
    return ttyopen(dev, tp);
}

/*ARGSUSED*/
int uartclose(dev_t dev, int flag, int mode) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];
    
    if (!tp->t_addr)
	return ENODEV;

    ttywflush(tp);
    ttyclose(tp);
    return (0);
}
/*ARGSUSED*/
int uartread(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    if (!tp->t_addr)
	return ENODEV;

    return ttread(tp, uio, flag);
}

/*ARGSUSED*/
int uartwrite(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    if (!tp->t_addr)
	return ENODEV;

    return ttwrite(tp, uio, flag);
}

int uartselect(dev_t dev, int rw) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    if (!tp->t_addr)
	return ENODEV;

    return (ttyselect(tp, rw));
}

/*ARGSUSED*/
int uartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];
    register int error;

    if (!tp->t_addr)
	return ENODEV;

    error = ttioctl(tp, cmd, addr, flag);
    if (error < 0)
	error = ENOTTY;
    return (error);
}

/* this receives the "status interrupt" which can be:
 *
 * UART_S1_TDRE      transmit data below watermark
 * UART_S1_TC        transmit complete
 * UART_S1_IDLE      idle line (what is this?)
 * UART_S1_RDRF      receive data above watermark
 * UART_S2_LBKDIF    LIN break detect (?)
 * UART_S2_RXEDGIF   rx pin active edge
 *
 */
void uartintr(dev_t dev) {
    register int c, s;
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];
    register struct uart_inst *uip;
    u_char head, tail, n, newhead, avail;

    if (!tp->t_addr)
	return;

    uip = (struct uart_inst *) tp->t_addr;

    /* receive data above watermark OR idle line */
    if (uip->inst->S1 & (UART_S1_RDRF | UART_S1_IDLE)) {
	/* disable irqs to avoid ending up back here with the underrun error  */
	s = spltty();

	avail = uip->inst->RCFIFO;
	/* next two comment blocks verbatim from original source */
	if (avail == 0) {
	    /* The only way to clear the IDLE interrupt flag is
	     * to read the data register.  But reading with no
	     * data causes a FIFO underrun, which causes the
	     * FIFO to return corrupted data.  If anyone from
	     * Freescale reads this, what a poor design!  There
	     * write should be a write-1-to-clear for IDLE.
	     */
	    c = uip->inst->D;
	    /* flushing the fifo recovers from the underrun,
	     * but there's a possible race condition where a
	     * new character could be received between reading
	     * RCFIFO == 0 and flushing the FIFO.  To minimize
	     * the chance, interrupts are disabled so a higher
	     * priority interrupt (hopefully) doesn't delay.
	     * TODO: change this to disabling the IDLE interrupt
	     * which won't be simple, since we already manage
	     * which transmit interrupts are enabled.
	     */
	    uip->inst->CFIFO = UART_CFIFO_RXFLUSH;
	    splx(s);
	} else {
	    splx(s);
	    head = uip->rx_buffer_head;
	    tail = uip->rx_buffer_tail;
	    do {
		n = uip->inst->D;
		newhead = head + 1;
		if (newhead >= RX_BUFFER_SIZE) newhead = 0;
		if (newhead != tail) {
		    head = newhead;
		    if (newhead < RX_BUFFER_SIZE) {
			uip->rx_buffer[head] = n;
                    } else {
                        uip->rx_buffer[head - RX_BUFFER_SIZE] = n;
                    }
                }
		/* let's consume the characters as they arrive? no idea how this works */
		ttyinput(uartgetc(dev), tp);
            } while (--avail > 0);
            uip->rx_buffer_head = head;
        }
    }
    c = uip->inst->C2;
    /* TDRE = transmit data below watermark */
    if ((c & UART_C2_TIE) && (uip->inst->S1 & UART_S1_TDRE)) {
	head = uip->tx_buffer_head;
	tail = uip->tx_buffer_tail;
	do {
	    if (tail == head) break;
	    if (++tail >= TX_BUFFER_SIZE) tail = 0;
	    avail = uip->inst->S1;
            if (tail < TX_BUFFER_SIZE) {
                n = uip->tx_buffer[tail];
            } else {
                n = uip->tx_buffer[tail - TX_BUFFER_SIZE];
            }
            uip->inst->D = n;
        } while (uip->inst->TCFIFO < 8);
        uip->tx_buffer_tail = tail;
	
        if (uip->inst->S1 & UART_S1_TDRE) /* we are soon done */
            uip->inst->C2 = C2_TX_COMPLETING;
    }

    /* yes, we are done. */
    if ((c & UART_C2_TCIE) && (uip->inst->S1 & UART_S1_TC)) {
        transmitting &= ~(1 << unit);
        /* TODO: check reference */
        uip->inst->C2 = C2_TX_INACTIVE;

	/* clear busy state as there is nothing left to send. */
	if (tp->t_state & TS_BUSY) {
	    tp->t_state &= ~TS_BUSY;
	    ttstart(tp);
	}
    }
}


/*
 * Start (restart) transmission on the given line.
 */
void uartstart(struct tty *tp) {
    register struct uart_inst *uip;
    register int c, s;

    if (!tp->t_addr)
	return;

    uip = (struct uart_inst *) tp->t_addr;

    /*
     * Must hold interrupts in following code to prevent
     * state of the tp from changing.
     */
    s   = spltty();
    /*
     * If it is currently active, or delaying, no need to do anything.
     */
    if (tp->t_state & (TS_TIMEOUT | TS_BUSY | TS_TTSTOP)) {
    out:
	led_control(LED_TTY, 0);
	splx(s);
	return;
    }

    /*
     * Wake up any sleepers.
     */
    ttyowake(tp);

    /*
     * Now restart transmission unless the output queue is empty.
     */
    if (tp->t_outq.c_cc == 0)
	goto out;

    /* is the transmit data register empty? if it is, let's go */
    if (uip->inst->S1 & UART_S1_TDRE) {
	c = getc(&tp->t_outq);
	uartputc(tp->t_dev, c);
	tp->t_state |= TS_BUSY;
    }

    led_control(LED_TTY, 1);
    splx(s);
}

void uartputc(dev_t dev, char c) {
    int unit                       = minor(dev);
    struct tty *tp                 = &uartttys[unit];
    register struct uart_inst *uip = &uart[unit];
    u_char n, head;
    register int s, timo;

    s = spltty();
    led_control(LED_TTY, 1);

    head = uip->tx_buffer_head;

    if (++head >= TX_BUFFER_SIZE) head = 0;
    while (uip->tx_buffer_tail == head) {
	if (uip->inst->S1 & UART_S1_TDRE) {
	    u_char tail = uip->tx_buffer_tail;
	    if (++tail >= TX_BUFFER_SIZE) tail = 0;
	    if (tail < TX_BUFFER_SIZE) {
		n = uip->tx_buffer[tail];
	    } else {
		n = uip->tx_buffer[tail-TX_BUFFER_SIZE];
	    }
	    uip->inst->D = n;
	    uip->tx_buffer_tail = tail;
	}
    }
    if (head < TX_BUFFER_SIZE) {
	uip->tx_buffer[head] = c;
    } else {
	uip->tx_buffer[head - TX_BUFFER_SIZE] = c;
    }
    transmitting &= (1 << unit);
    uip->tx_buffer_head = head;
    uip->inst->C2 = C2_TX_ACTIVE;

    led_control(LED_TTY, 0);
    splx(s);
}


char uartgetc(dev_t dev) {
    int unit                       = minor(dev);
    register struct uart_inst *uip = &uart[unit];
    int s, c;
    u_char head, tail;

    s = spltty();
    
    head = uip->rx_buffer_head;
    tail = uip->rx_buffer_tail;

    if (head == tail) {
	while (1) {
	    ; /* TODO: do whatever is supposed to be done */
	}
    }
    if (++tail >= RX_BUFFER_SIZE) tail = 0;
    if (tail < RX_BUFFER_SIZE) {
	c = uip->rx_buffer[tail];
    } else {
	c = uip->rx_buffer[tail - RX_BUFFER_SIZE];
    }
    uip->rx_buffer_tail = tail;

    splx(s);
    return (unsigned char) c;
}

/*
 * Test to see if device is present.
 * Return true if found and initialized ok.
 */
static int
uartprobe(struct conf_device *config)
{
    int unit = config->dev_unit - 1;
    int is_console = (CONS_MAJOR == UART_MAJOR &&
		      CONS_MINOR == unit);

    if (unit < 0 || unit >= NUART)
	return 0;

    switch (unit) {
    case 0:
	SIM_SCGC4 |= SIM_SCGC4_UART0;
	break;
    default:
	printf("uart: unit %d not supported\n");
	return 0;
    }

    transmitting &= ~(1 << unit);
    
    printf("uart%d: rx pin %d, tx pin %d\n",
	   unit+1,
	   uart[unit].rx_pin_num, uart[unit].tx_pin_num);

    if (is_console)
	printf(", console");
    printf("\n");

    /* Initialize the device. */
    uartttys[unit].t_addr = (caddr_t) &uart[unit];
    if (! is_console)
	uartinit(unit);

    return 1;
}

struct driver uartdriver = {
    "uart",
    uartprobe,
};
