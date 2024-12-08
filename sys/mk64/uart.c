/** UART driver for Kinesis K / Teensy.
 *
 *
 *
 ** REGISTERS
 *
 ** S1 - status register 1
 *
 * UART_S1_TDRE            0x80                Transmit Data Register Empty Flag
 * UART_S1_TC              0x40                Transmit Complete Flag
 * UART_S1_RDRF            0x20                Receive Data Register Full Flag
 * UART_S1_IDLE            0x10                Idle Line Flag
 * UART_S1_OR              0x08                Receiver Overrun Flag
 * UART_S1_NF              0x04                Noise Flag
 * UART_S1_FE              0x02                Framing Error Flag
 * UART_S1_PF              0x01                Parity Error Flag
 *
 *
 ** S2 - status register 2
 *
 * UART_S2_LBKDIF          0x80                LIN Break Detect Interrupt Flag
 * UART_S2_RXEDGIF         0x40                RxD Pin Active Edge Interrupt Flag
 * UART_S2_MSBF            0x20                Most Significant Bit First
 * UART_S2_RXINV           0x10                Receive Data Inversion
 * UART_S2_RWUID           0x08                Receive Wakeup Idle Detect
 * UART_S2_BRK13           0x04                Break Transmit Character Length
 * UART_S2_LBKDE           0x02                LIN Break Detection Enable
 * UART_S2_RAF             0x01                Receiver Active Flag
 *
 *
 ** C1 - control register 1
 *
 * UART_C1_LOOPS           0x80                Enable loopback
 * UART_C1_UARTSWAI        0x40                UART Stops in Wait Mode
 * UART_C1_RSRC            0x20                When LOOPS is set, the RSRC field determines
 *                                             the source for the receiver shift register input
 * UART_C1_M               0x10                9-bit or 8-bit Mode Select
 * UART_C1_WAKE            0x08                Determines which condition wakes the UART
 * UART_C1_ILT             0x04                Idle Line Type Select
 * UART_C1_PE              0x02                Parity Enable
 * UART_C1_PT              0x01                Parity Type, 0=even, 1=odd
 *
 ** C2 - control register 2
 *
 *
 * UART_C2_TIE             0x80                Transmitter Interrupt or DMA Transfer Enable.
 * UART_C2_TCIE            0x40                Transmission Complete Interrupt Enable
 * UART_C2_RIE             0x20                Receiver Full Interrupt or DMA Transfer Enable
 * UART_C2_ILIE            0x10                Idle Line Interrupt Enable
 * UART_C2_TE              0x08                Transmitter Enable
 * UART_C2_RE              0x04                Receiver Enable
 * UART_C2_RWU             0x02                Receiver Wakeup Control
 * UART_C2_SBK             0x01                Send Break
 *
 *
 ** C3 - control register 3
 *
 * UART_C3_R8              0x80                Received Bit 8
 * UART_C3_T8              0x40                Transmit Bit 8
 * UART_C3_TXDIR           0x20                TX Pin Direction in Single-Wire mode
 * UART_C3_TXINV           0x10                Transmit Data Inversion
 * UART_C3_ORIE            0x08                Overrun Error Interrupt Enable
 * UART_C3_NEIE            0x04                Noise Error Interrupt Enable
 * UART_C3_FEIE            0x02                Framing Error Interrupt Enable
 * UART_C3_PEIE            0x01                Parity Error Interrupt Enable
 *
 *
 ** D - data registers, read/write aliased behind the same address
 *
 *
 ** PFIFO - FIFO parameter register
 * 
 * UART_PFIFO_TXFE         0x80                Transmit FIFO Enable
 * UART_PFIFO_TXFIFOSIZE(n) (((n) & 7) << 4)   Transmit FIFO Size - if !0, sz = n<<2
 *                                             0=1, 1=4, 2=8, 3=16, 4=32, 5=64, 6=128
 * UART_PFIFO_RXFE         0x08                Receive FIFO Enable
 * UART_PFIFO_RXFIFOSIZE(n) (((n) & 7) << 0)   Receive FIFO Size
 *
 *
 ** CFIFO - FIFO control register
 *
 * UART_CFIFO_TXFLUSH      0x80                 Transmit FIFO/Buffer Flush
 * UART_CFIFO_RXFLUSH      0x40                 Receive FIFO/Buffer Flush
 * UART_CFIFO_RXOFE        0x04                 Receive FIFO Overflow Interrupt Enable
 * UART_CFIFO_TXOFE        0x02                 Transmit FIFO Overflow Interrupt Enable
 * UART_CFIFO_RXUFE        0x01                 Receive FIFO Underflow Interrupt Enable
 *
 *
 ** SFIFO - FIFO status register
 *
 * UART_SFIFO_TXEMPT       0x80                 Transmit Buffer/FIFO Empty
 * UART_SFIFO_RXEMPT       0x40                 Receive Buffer/FIFO Empty
 * UART_SFIFO_RXOF         0x04                 Receiver Buffer Overflow Flag
 * UART_SFIFO_TXOF         0x02                 Transmitter Buffer Overflow Flag
 * UART_SFIFO_RXUF         0x01                 Receiver Buffer Underflow Flag
 *
 *
 ** TWFIFO - transmit FIFO watermark
 ** TCFIFO - transmit FIFO count
 ** RWFIFO - receive FIFO watermark
 ** RCFIFO - receive FIFO count
 *
 * K64 Sub-Family Reference Manual, Rev. 2, January 2014, pages 1609-1610
 * 52.8.3 Initialization sequence (non ISO-7816)
 *
 * To initiate a UART transmission:
 *
 * 1. Configure the UART.
 *
 *  a. Select a baud rate. Write this value to the UART baud registers
 *     (BDH/L) to begin the baud rate generator. Remember that the
 *     baud rate generator is disabled when the baud rate is
 *     zero. Writing to the BDH has no effect without also writing to
 *     BDL.
 *
 *  b. Write to C1 to configure word length, parity, and other configuration bits
 *     (LOOPS, RSRC, M, WAKE, ILT, PE, and PT). Write to C4, MA1, and MA2 to
 *     configure.
 * 
 *  c. Enable the transmitter, interrupts, receiver, and wakeup as
 *     required, by writing to C2 (TIE, TCIE, RIE, ILIE, TE, RE, RWU,
 *     and SBK), S2 (MSBF and BRK13), and C3 (ORIE, NEIE, PEIE, and
 *     FEIE). A preamble or idle character is then shifted out of the
 *     transmitter shift register.
 * 
 * 2. Transmit procedure for each byte.
 *
 *  a. Monitor S1[TDRE] by reading S1 or responding to the TDRE interrupt. The
 *     amount of free space in the transmit buffer directly using TCFIFO[TXCOUNT]
 *     can also be monitored.
 * 
 *  b. If the TDRE flag is set, or there is space in the transmit
 *     buffer, write the data to be transmitted to (C3[T8]/D). A new
 *     transmission will not result until data exists in the transmit
 *     buffer.
 *
 *  3. Repeat step 2 for each subsequent transmission.
 * 
 *      Note
 * 
 *       During normal operation, S1[TDRE] is set when the shift
 *       register is loaded with the next data to be transmitted from the
 *       transmit buffer and the number of datawords contained in the
 *       transmit buffer is less than or equal to the value in
 *       TWFIFO[TXWATER]. This occurs 9/16ths of a bit time after
 *       the start of the stop bit of the previous frame.
 *
 *  To separate messages with preambles with minimum idle line time,
 *  use this sequence between messages.
 *
 *  1. Write the last dataword of the first message to C3[T8]/D.
 *
 *  2. Wait for S1[TDRE] to go high with TWFIFO[TXWATER] = 0, indicating the
 *     transfer of the last frame to the transmit shift register.
 *
 *  3. Queue a preamble by clearing and then setting C2[TE].
 *
 *  4. Write the first and subsequent datawords of the second message to C3[T8]/D.
 *
 *
 */

/*
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
#include <machine/teensy.h>
#include <machine/gpio.h>
#include <machine/systick.h>

#define CONCAT(x, y) x##y
#define BBAUD(x)     CONCAT(B, x)

#ifndef UART_BAUD
#define UART_BAUD 38400
#endif

struct uart_inst {
    volatile KINETISK_UART_t *regs;
    u_char rx_pin_num;
    u_char tx_pin_num;
    int fifosz;
    /* interrupt tally */
    u_int isrcnt;
    u_int errisrcnt;
    /* overflows */
    u_int rxofcnt;
    u_int txofcnt;
    /* other interrupts by type */
    u_int tdrecnt; 
    u_int tccnt;
    u_int rdrfcnt;
    u_int idlecnt;
};

static struct uart_inst uart[NUART] = {
#ifdef TEENSY35
    {&KINETISK_UART0, 16, 17},
    {&KINETISK_UART1, 3, 4},
#endif
};

struct tty uartttys[NUART];

static unsigned speed_bps [NSPEEDS] = {
    0,       50,      75,      150,     200,    300,     600,     1200,
    1800,    2400,    4800,    9600,    19200,  38400,   57600,   115200,
    230400,  460800,  500000,  576000,  921600, 1000000, 1152000, 1500000,
    2000000, 2500000, 3000000, 3500000, 4000000
};
/*
 * as NUART is at most less than 8,
 * we can get away with these for now.
 */
static volatile uint8_t alive;

void cnstart (struct tty *tp);

void uart0_status_isr(void) {
    uart[0].isrcnt++;

    if (alive & 1) {
        uartintr(makedev(UART_MAJOR, 0));
    } else {
	printf("uart0: stray interrupt\n");
    }
}

void uart0_error_isr(void) {
    uart[0].errisrcnt++;
}

void uart1_status_isr(void) {
    uart[1].isrcnt++;
    if (alive & (1 << 1)) {
        uartintr(makedev(UART_MAJOR, 1));
    } else {
	printf("uart1: stray interrupt\n");
    }
}

void uart1_error_isr(void) {
    uart[1].errisrcnt++;
}

void uartinit(int unit) {
    register struct uart_inst *inst;
    int divisor;

    if (unit > 1) {
        printf("uart: unit %d not supported yet\n", unit);
        return;
    }

    inst = &uart[unit];

    inst->isrcnt = inst->errisrcnt = 0;
    inst->txofcnt = inst->rxofcnt = 0;
    inst->tdrecnt = inst->tccnt = inst->rdrfcnt = inst->idlecnt = 0;

    switch (unit) {
    case 0:
	SIM_SCGC4 |= SIM_SCGC4_UART0;

        arm_disable_irq(IRQ_UART0_STATUS);
        arm_disable_irq(IRQ_UART0_ERROR);

	PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
	    PORT_PCR_MUX(3);
	PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
    case 1:
	SIM_SCGC4 |= SIM_SCGC4_UART1;

        arm_disable_irq(IRQ_UART1_STATUS);
        arm_disable_irq(IRQ_UART1_ERROR);

	PORTC_PCR3 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
	    PORT_PCR_MUX(3);
	PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
    }

    /* fixed speed for now. */

    inst->regs->BDH    = 0;
    inst->regs->BDL    = 0xc3; /* dec 65 = 115200, thus dec 780 = 9600,
				  means BDH 3, BDL c */
    inst->regs->C4     = 0b01010; /* 0, 0xc3, 0b01010 gives perfect 38400 */
    inst->regs->C2     = 0;
    inst->regs->S2     = 0;
	
    inst->regs->C1     = UART_C1_ILT;
    inst->regs->C5     = 0;

    inst->regs->PFIFO  = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
    /* interrupt on both overflows but not on receive underflow
     * because of how the UART is designed - see quoted comment in
     * interrupt handler
     */
    inst->regs->CFIFO = (UART_CFIFO_RXOFE | UART_CFIFO_TXOFE); 

    inst->fifosz = 8;

    inst->regs->TWFIFO = 1; // tx watermark, causes S1_TDRE to set
    inst->regs->RWFIFO = 2; // rx watermark, causes S1_RDRF to set

    /*
     * deep breath, hope for the best and activate isr
     */
    inst->regs->C2     = C2_ENABLE;

    switch(unit) {
    case 0:
	arm_set_irq_prio(IRQ_UART0_STATUS, SPL_TTY);
	arm_set_irq_prio(IRQ_UART0_ERROR, SPL_TTY);
	arm_enable_irq(IRQ_UART0_ERROR);
	arm_enable_irq(IRQ_UART0_STATUS);
	break;
    case 1:
	arm_set_irq_prio(IRQ_UART1_STATUS, SPL_TTY);
	arm_set_irq_prio(IRQ_UART1_ERROR, SPL_TTY);
	arm_enable_irq(IRQ_UART1_ERROR);
	arm_enable_irq(IRQ_UART1_STATUS);
	break;
    }

    alive |= (1 << unit);
}

int uartopen(dev_t dev, int flag, int mode) {
    register struct uart_inst *uip;
    register struct tty *tp;
    register int unit = minor(dev);
    int error, s;

    if (unit < 0 || unit >= NUART)
        return (ENXIO);
    
    s = spltty();

    tp = &uartttys[unit];

    tp->t_oproc = uartstart;
    uip = &uart[unit];
    tp->t_addr = (caddr_t) uip;

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
    if ((tp->t_state & TS_XCLUDE) && u.u_uid != 0) {
        error = (EBUSY);
	goto out;
    }

    error = ttyopen(dev, tp);
    
 out:
    splx(s);
    return error;
}

/*ARGSUSED*/
int uartclose(dev_t dev, int flag, int mode) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    ttywflush(tp);
    ttyclose(tp);

    
    return (0);
}

/*ARGSUSED*/
int uartread(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    return ttread(tp, uio, flag);
}

/*ARGSUSED*/
int uartwrite(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    return ttwrite(tp, uio, flag);
}

int uartselect(dev_t dev, int rw) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];

    return (ttyselect(tp, rw));
}

/*ARGSUSED*/
int uartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];
    register int error;

    error = ttioctl(tp, cmd, addr, flag);
    if (error < 0)
        error = ENOTTY;
    return (error);
}

/** Kinetis K UART interrupt handler
 *
 * this receives the "status interrupt" which can be:
 *
 * UART_S1_TDRE      transmit data below watermark
 *
 * "To clear TDRE, read S1 when TDRE is set and then write to the UART
 * data register (D). For more efficient interrupt servicing, all data
 * except the final value to be written to the buffer must be written
 * to D/C3[T8]. Then S1 can be read before writing the final data
 * value, resulting in the clearing of the TRDE flag. This is more
 * efficient because the TDRE reasserts until the watermark has been
 * exceeded. So, attempting to clear the TDRE with every write will be
 * ineffective until sufficient data has been written."
 *
 * UART_S1_TC        transmit complete
 *
 * "TC is cleared by reading S1 with TC set and then doing one of
 * the following: 
 *
 * • Writing to D to transmit new data.
 * • Queuing a preamble by clearing and then setting C2[TE].
 * • Queuing a break character by writing 1 to SBK in C2.
 *
 * UART_S1_IDLE      idle line 
 *
 * "After the IDLE flag is cleared, a frame must be received (although
 * not necessarily stored in the data buffer, for example if C2[RWU]
 * is set), or a LIN break character must set the S2[LBKDIF] flag
 * before an idle condition can set the IDLE flag. To clear IDLE, read
 * UART status S1 with IDLE set and then read D.  IDLE is set when
 * either of the following appear on the receiver input:
 *
 * • 10 consecutive logic 1s if C1[M] = 0
 * • 11 consecutive logic 1s if C1[M] = 1 and C4[M10] = 0
 * • 12 consecutive logic 1s if C1[M] = 1, C4[M10] = 1, and C1[PE] = 1"
 *
 * UART_S1_RDRF      receive data above watermark
 *
 * "To clear RDRF, read S1 when RDRF is set and then read D. For more
 * efficient interrupt and DMA operation, read all data except the
 * final value from the buffer, using D/C3[T8]/ED. Then read S1 and
 * the final data value, resulting in the clearing of the RDRF
 * flag. Even if RDRF is set, data will continue to be received until
 * an overrun condition occurs.RDRF is prevented from setting while
 * S2[LBKDE] is set.  Additionally, when S2[LBKDE] is set, the
 * received datawords are stored in the receive buffer but over-write
 * each other."
 *
 * UART_S1_TIE       transmitter interrupt
 * UART_S2_LBKDIF
 * UART_S2_RXEDGIF   receive edge detection
 * 
 *
 **
 * the C2-related macros are borrowed from Teensyduino. they enable
 * different interrupts and features as follows:
 *
 * C2_ENABLE         enable transmitter & receiver,
 *                   RIE (receiver full) interrupt and ILIE (idle line) interrupt
 *
 * C2_TX_ACTIVE      C2_ENABLE and TIE (transmitter) interrupt
 *                   set by tx fifo level going over the watermark
 *
 * C2_TX_COMPLETING  C2_ENABLE and TCIE (transmission complete) interrupt
 *                   set by tx fifo level going under the watermark
 *
 * C2_TX_INACTIVE    same as C2_ENABLE
 *
 *
 * 
 */
void uartintr(dev_t dev) {
    register int c, s;
    register int unit       = minor(dev);
    register struct tty *tp = &uartttys[unit];
    register struct uart_inst *uip;
    u_char n, avail;

    s   = spltty();

    teensy_gpio_led_value(0x80 + unit);
    
    uip = (struct uart_inst *) &uart[unit];

    /* receive data above watermark or idle line */
    if (uip->regs->S1 & (UART_S1_RDRF | UART_S1_IDLE)) {
        avail = uip->regs->RCFIFO;
        /* next two comment blocks verbatim from original source */
        if (avail == 0) {
	    uip->idlecnt++;
            /* The only way to clear the IDLE interrupt flag is
	     * to read the data register.  But reading with no
	     * data causes a FIFO underrun, which causes the
	     * FIFO to return corrupted data.  If anyone from
	     * Freescale reads this, what a poor design!  There
	     * write should be a write-1-to-clear for IDLE.
	     */
            c           = uip->regs->D;
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
	    uip->regs->CFIFO = UART_CFIFO_RXFLUSH;
        } else {
	    uip->rdrfcnt++;
	    
            /* while (avail--) { */
	    while (! (uip->regs->SFIFO & UART_SFIFO_RXEMPT)) {
                n = uip->regs->D;
                ttyinput(n, tp);
            }
        }

    }
    
    /* update receive overflow counter and clear the flag */
    if ((uip->regs->SFIFO & UART_SFIFO_RXOF)) {
	uip->rxofcnt++;
	uip->regs->SFIFO |= ~UART_SFIFO_RXOF;
    }
    c = uip->regs->C2;

    /* TIE = transmitter interrupt, TDRE = transmit data below watermark */
    if ((c & UART_C2_TIE) && (uip->regs->S1 & UART_S1_TDRE)) {
	uip->tdrecnt++;
	avail = uip->regs->S1; /* value not used, part of interrupt clearing dance */

        if (tp->t_outq.c_cc) {
	    while (tp->t_outq.c_cc && uip->regs->TCFIFO < (uip->fifosz - 1)) {
		uip->regs->D = getc(&tp->t_outq);
	    }
        } else if (uip->regs->S1 & UART_S1_TDRE) {
            uip->regs->C2 = C2_TX_COMPLETING; /* ignore TIE, wait for TCIE */
        }

    }
    
    /* TC = transmit complete interrupt */
    if ((c & UART_C2_TCIE) && (uip->regs->S1 & UART_S1_TC)) {
	uip->tccnt++;

	    #if 0
	if (tp->t_outq.c_cc) {
	    /* nope, it's not over yet... */
	    while (tp->t_outq.c_cc && uip->regs->TCFIFO < (uip->fifosz - 1)) {
		uip->regs->D = getc(&tp->t_outq);
	    }
	    uip->regs->C2 = C2_TX_ACTIVE;
	} else {
	    #endif
	    /* okay, it IS over... turn the thing off, then! */
	    uip->regs->C2 = C2_TX_INACTIVE;
	    
	    if (tp->t_state & TS_BUSY) {
		tp->t_state &= ~TS_BUSY;
	    }
	    #if 0
	}
	#endif
	/* ...maybe refill the FIFO here instead? */
	ttstart(tp);
    }
    if ((uip->regs->SFIFO & UART_SFIFO_TXOF)) {
	uip->txofcnt++;
	uip->regs->SFIFO |= ~UART_SFIFO_TXOF;
    }
    splx(s);
}


void uartstart(struct tty *tp) {
    register struct uart_inst *uip;
    register int c, s;

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

    tp->t_state |= TS_BUSY;
    while(tp->t_outq.c_cc) {
	if (uip->regs->TCFIFO < uip->regs->TWFIFO) {
            uip->regs->D  = getc(&tp->t_outq);
	}
    }
    uip->regs->C2 = C2_TX_ACTIVE;
    splx(s);
}

void uartputc(dev_t dev, char c) {
    int unit                       = minor(dev);
    struct tty *tp                 = &uartttys[unit];
    register struct uart_inst *uip = &uart[unit];
    int s, timo;

    timo = 10000;

    while (! (uip->regs->SFIFO & UART_SFIFO_TXEMPT)) {
	if(--timo == 0)
	    break;
    }

    if (c == 0)
	return;
    
    uip->regs->D = c;
    uip->regs->C2 = C2_TX_ACTIVE;

    uartputc(dev, 0);
}

void uartstat(void) {
    for (int i = 0; i < NUART; i++) {
	printf("\nuart%d: isr %d errisr %d rxof %d txof %d tdre %d tc %d rdrf %d idle %d",
	       i,
	       uart[i].isrcnt, uart[i].errisrcnt,
	       uart[i].rxofcnt, uart[i].txofcnt,
	       uart[i].tdrecnt, uart[i].tccnt,
	       uart[i].rdrfcnt, uart[i].idlecnt);
    }
    printf("\n");
}

char uartgetc(dev_t dev) {
    int unit                       = minor(dev);
    register struct uart_inst *uip = &uart[unit];
    int s, c;
 
    s = spltty();

    for (;;) {
        if (uip->regs->RCFIFO) {
            c = uip->regs->D;
            break;
        }
	uartintr(dev);
    }

    splx(s);
    
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

    /* Initialize the device. */
    uartttys[unit].t_addr = (caddr_t) &uart[unit];
    if (!is_console)
        uartinit(unit);
    
    printf("uart%d: %d-byte FIFO", unit + 1, (unit == 0 || unit == 1) ? 8 : 1);

    if (is_console)
        printf(", console");
    printf("\n");

    return 1;
}

struct driver uartdriver = {
    "uart",
    uartprobe,
};
