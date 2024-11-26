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

#include <machine/debug.h>
#include <machine/uart.h>
#include <machine/kinetis.h>
#include <machine/intr.h>

#define CONCAT(x, y) x##y
#define BBAUD(x)     CONCAT(B, x)

#ifndef UART_BAUD
#define UART_BAUD 115200
#endif

struct uart_inst {
    volatile KINETISK_UART_t *regs;
    u_char rx_pin_num;
    u_char tx_pin_num;
    volatile u_int *regbase; /* 00 pdor - can be 20, 40, 80, ...
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
    {&KINETISK_UART0, 16, 17, &GPIOB_PDOR},
#if 0
    {&(*(KINETISK_UART_t *) UART1_ADDR)},
    {&(*(KINETISK_UART_t *) UART2_ADDR)},
    {&(*(KINETISK_UART_t *) UART3_ADDR)},
    {&(*(KINETISK_UART_t *) UART4_ADDR)},
    {&(*(KINETISK_UART_t *) UART5_ADDR)}
#else
    {},
    {},
    {},
    {},
    {}
#endif

#endif
};

struct tty uartttys[NUART];

/*
 * as NUART is at most less than 8,
 * we can get away with this. 
 */
static volatile uint8_t transmitting;
static volatile uint8_t alive;

void uart0_status_isr(void) {
    if (alive == 1)
        uartintr(makedev(UART_MAJOR, 0));
}
void uart0_error_isr(void) {
    if (alive == 1)
        uartintr(makedev(UART_MAJOR, 0));
}

void uartinit(int unit) {
    struct uart_inst *inst;
    int divisor, c, s;

    s = arm_disable_interrupts();

    SIM_SCGC4 |= SIM_SCGC4_UART0;

    if (unit != 0) {
        printf("uart: unit != 0 not supported yet\n");
        return;
    }

    inst = &uart[unit];

    switch (unit) {
    case 0:
    case 1:
        arm_disable_irq(IRQ_UART0_STATUS);
        arm_disable_irq(IRQ_UART0_ERROR);

        if (unit == 0) {
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
        if (divisor < 32)
            divisor = 32;

#if 0
        UART0_BDH    = (divisor >> 13) & 0x1F;
        UART0_BDL    = (divisor >> 5) & 0xFF;
        UART0_C4     = divisor & 0x1F;
#endif
        UART0_BDH    = 0;
        UART0_BDL    = 65;
        UART0_C4     = 0b00011; /* this or 0b00011 */

        UART0_C1     = UART_C1_ILT;
        UART0_C2     = C2_TX_INACTIVE;

        UART0_TWFIFO = 2; // tx watermark, causes S1_TDRE to set
        UART0_RWFIFO = 4; // rx watermark, causes S1_RDRF to set

        UART0_PFIFO  = UART_PFIFO_TXFE | UART_PFIFO_RXFE |
                      UART_PFIFO_TXFIFOSIZE(6) |
                      UART_PFIFO_RXFIFOSIZE(6);

        c        = UART0_C1;
        c        = (c & ~0x13) | (SERIAL_8N1 & 0x03);
        UART0_C1 = c;

        /*
	 * deep breath, hope for the best and activate isr
	 */
        arm_set_irq_prio(IRQ_UART0_STATUS, SPL_TTY);
        arm_set_irq_prio(IRQ_UART0_ERROR, SPL_TTY);

        alive = 1;

        arm_enable_irq(IRQ_UART0_ERROR);
        arm_enable_irq(IRQ_UART0_STATUS);


        break;
    default:
        printf("uart: wtf\n");
        break;
    }

    arm_restore_interrupts(s);
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
        tp->t_flags =
            ECHO | XTABS | CRMOD | CRTBS | CRTERA | CTLECH | CRTKIL;
    }
    if ((tp->t_state & TS_XCLUDE) && u.u_uid != 0)
        return (EBUSY);

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

    s   = spltty();

    /* receive data above watermark OR idle line */
    if (UART0_S1 & (UART_S1_RDRF | UART_S1_IDLE)) {
        /* disable irqs to avoid ending up back here with the underrun error  */
        arm_disable_irq(IRQ_UART0_STATUS);
        arm_disable_irq(IRQ_UART0_ERROR);

        avail = UART0_RCFIFO;
        /* next two comment blocks verbatim from original source */

        if (avail == 0) {
            /* The only way to clear the IDLE interrupt flag is
	     * to read the data register.  But reading with no
	     * data causes a FIFO underrun, which causes the
	     * FIFO to return corrupted data.  If anyone from
	     * Freescale reads this, what a poor design!  There
	     * write should be a write-1-to-clear for IDLE.
	     */
            c           = UART0_D;
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
            UART0_CFIFO = UART_CFIFO_RXFLUSH;
            arm_enable_irq(IRQ_UART0_STATUS);
            arm_enable_irq(IRQ_UART0_ERROR);
            splx(s);
        } else {
            arm_enable_irq(IRQ_UART0_STATUS);
            arm_enable_irq(IRQ_UART0_ERROR);
            splx(s);

            n = UART0_D;
            ttyinput(n, tp);
        }
    }
    c = UART0_C2;
    /* TIE = transmitter interrupt, TDRE = transmit data below watermark */
    if ((c & UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE)) {
        if (tp->t_outq.c_cc) {
            tp->t_state &= ~TS_BUSY;

            do {
                UART0_D = getc(&tp->t_outq);
            } while (UART0_TCFIFO < 8);
        }

        if (UART0_S1 & UART_S1_TDRE)
            UART0_C2 = C2_TX_COMPLETING;
        else
            UART0_C2 = C2_TX_ACTIVE;
    }
    /* TC = transmit complete - yes, we are done. */
    if ((c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {
        transmitting &= ~(1 << unit);
        /* TODO: check reference */
        UART0_C2 = C2_TX_INACTIVE;

        /* clear busy state as there is nothing left to send. */
        if (tp->t_state & TS_BUSY) {
            tp->t_state &= ~TS_BUSY;
            ttstart(tp);
        }
    }
    splx(s);
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

    s   = spltty();
    /*
     * let's try again once the terminal is not in this state.
     */
    if (tp->t_state & (TS_TIMEOUT | TS_BUSY | TS_TTSTOP)) {
    out:
        splx(s);
        return;
    }

    ttyowake(tp);

    if (tp->t_outq.c_cc == 0)
        goto out;

    /* is the transmit data register empty? if it is, let's go */
    if (UART0_S1 & UART_S1_TDRE) {
        c        = getc(&tp->t_outq);
        UART0_D  = c;
        UART0_C2 = C2_TX_ACTIVE;
        tp->t_state |= TS_BUSY;
    }

    splx(s);
}

void uartputc(dev_t dev, char c) {
    int unit                       = minor(dev);
    struct tty *tp                 = &uartttys[unit];
    register struct uart_inst *uip = &uart[unit];
    register int s, try;

    s   = spltty();

    try = 3;

again:
    if (UART0_TCFIFO < 100)
        UART0_D = c;
    else {
        tp->t_state |= TS_BUSY;
        uartintr(dev);
    }

    if (try && (tp->t_state & TS_BUSY)) {
        uartintr(dev);
        try--;
        goto again;
    }

    splx(s);
}


char uartgetc(dev_t dev) {
    int unit                       = minor(dev);
    register struct uart_inst *uip = &uart[unit];
    int s, c;

    for (;;) {
        if (UART0_RCFIFO) {
            c = UART0_D;
            break;
        }
    }

    return (unsigned char) c;
}

/*
 * Test to see if device is present.
 * Return true if found and initialized ok.
 */
static int uartprobe(struct conf_device *config) {
    int unit       = config->dev_unit - 1;
    int is_console = (CONS_MAJOR == UART_MAJOR && CONS_MINOR == unit);

    if (unit < 0 || unit >= NUART)
        return 0;

    switch (unit) {
    case 0:
        break;
    default:
        printf("uart: unit %d not supported\n");
        return 0;
    }

    transmitting &= ~(1 << unit);

    printf("uart%d: rx pin %d, tx pin %d", unit + 1,
           uart[unit].rx_pin_num, uart[unit].tx_pin_num);

    if (is_console)
        printf(", console");
    printf("\n");

    /* Initialize the device. */
    uartttys[unit].t_addr = (caddr_t) &uart[unit];
    if (!is_console)
        uartinit(unit);

    return 1;
}

struct driver uartdriver = {
    "uart",
    uartprobe,
};
