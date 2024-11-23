/*
 * Teensy 3.5 virtual serial over USB. For now, only one.
 *
 * Copyright (c) 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/user.h>
#include <sys/ioctl.h>
#include <sys/tty.h>
#include <sys/systm.h>
#include <sys/kconfig.h>

#include <machine/uartusb.h>
#include <machine/teensy_usb_serial.h>

struct tty usbuartttys[1];

void cnstart(struct tty *tp);

struct uartusb_inst {
    volatile u_char rx_buffer[RX_BUFFER_SIZE];
    volatile u_char tx_buffer[TX_BUFFER_SIZE];
    volatile u_char rx_buffer_head;
    volatile u_char rx_buffer_tail;
    volatile u_char tx_buffer_head;
    volatile u_char tx_buffer_tail;
};

static struct uartusb_inst uartusb[1] = {{{0}, {0}, 0, 0, 0, 0}};

void uartusbinit(int unit) {
    register struct uartusb_inst *inst; 
    
    if (unit != 0)
        return;

    inst = &uartusb[unit];
}

int usbuartopen(dev_t dev, int flag, int mode) {
    register struct uartusb_inst *uip; 
    register struct tty *tp;
    register int unit = minor(dev);

    if (unit != 0)
        return (ENXIO);

    tp          = &usbuartttys[unit];

    uip         = (struct uartusb_inst *) tp->t_addr;
    tp->t_oproc = usbuartstart;
    if ((tp->t_state & TS_ISOPEN) == 0) {
        if (tp->t_ispeed ==
            0) { /* it's usb, the speed specified here is irrelevant */
            tp->t_ispeed = 115200;
            tp->t_ospeed = 115200;
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
int usbuartclose(dev_t dev, int flag, int mode) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    ttywflush(tp);
    ttyclose(tp);
    return (0);
}

/*ARGSUSED*/
int usbuartread(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return ttread(tp, uio, flag);
}

/*ARGSUSED*/
int usbuartwrite(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return ttwrite(tp, uio, flag);
}

int usbuartselect(dev_t dev, int rw) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return (ttyselect(tp, rw));
}

/*ARGSUSED*/
int usbuartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[unit];
    register int error;

    if (!tp->t_addr)
        return ENODEV;

    error = ttioctl(tp, cmd, addr, flag);
    if (error < 0)
        error = ENOTTY;
    return (error);
}

void usbuartintr(dev_t dev) {
    register int c, s;
    register int unit       = minor(dev);
    register struct tty *tp;
    register struct uartusb_inst *uip;
    u_char head, tail, n, newhead, avail;

    if (unit != 0)
	return;

    tp          = &usbuartttys[unit];

    if (!tp->t_addr)
        return;

    /* one day, a beautiful buffered implementation will arrive here.
     * that day is not today.
     */

    uip         = (struct uartusb_inst *) tp->t_addr;

    s = spltty();

    avail = usb_serial_available();

    if (avail) {
        do {
            ttyinput(usb_serial_getchar(), tp);
        } while (--avail > 0);
    }

    splx(s);
    
    if (tp->t_state & TS_BUSY) {
        tp->t_state &= ~TS_BUSY;
        ttstart(tp);
    }
}

/* usb_isr() from teensy_usb_dev.c calls this */
void usb_uart_isr(void) {
    usbuartintr(makedev(UARTUSB_MAJOR, 0));
}

/*
 * Start (restart) transmission on the given line.
 */
void usbuartstart(struct tty *tp) {
    register int c, s;

    if (!tp->t_addr)
        return;

    /*
     * Must hold interrupts in following code to prevent
     * state of the tp from changing.
     */
    s = spltty();
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

    do {
        c = getc(&tp->t_outq);
        usb_serial_putchar(c);
    } while (tp->t_outq.c_cc);

    tp->t_state &= ~TS_BUSY;

    led_control(LED_TTY, 1);
    splx(s);
}

void usbuartputc(dev_t dev, char c) {
    int unit       = minor(dev);
    struct tty *tp = &usbuartttys[unit];
    register int s, timo;

    s = spltty();
again:
    /*
     * Try waiting for the console tty to come ready,
     * otherwise give up after a reasonable time.
     */
#if 0
    timo = 30000;
    while (!LL_USART_IsActiveFlag_TXE(uip->inst))
        if (--timo == 0)
            break;
#endif

    if (tp->t_state & TS_BUSY) {
        usbuartintr(dev);
        goto again;
    }
    led_control(LED_TTY, 1);
    usb_serial_putchar(c);

#if 0
    timo = 30000;
    while (!LL_USART_IsActiveFlag_TC(uip->inst))
        if (--timo == 0)
            break;
#endif

    led_control(LED_TTY, 0);
    splx(s);
}

char usbuartgetc(dev_t dev) {
    int unit = minor(dev);
    int s, c;

    s = spltty();
    for (;;) {
        /* Wait for key pressed. */
        if (usb_serial_available()) {
            c = usb_serial_getchar();
            break;
        }
    }

    /* RXNE flag was cleared by reading DR register */

    splx(s);
    return (unsigned char) c;
}

/*
 * Test to see if device is present.
 * Return true if found and initialized ok.
 */
static int uartusbprobe(struct conf_device *config) {
    int unit       = config->dev_unit - 1;
    int is_console = (CONS_MAJOR == UARTUSB_MAJOR && CONS_MINOR == unit);

    if (unit != 0)
        return 0;

    printf("uartusb%d: USB", unit);

    if (is_console)
        printf(", console");
    printf("\n");

    /* Initialize the device. */
    usbuartttys[unit].t_addr = (caddr_t) &uartusb[unit];
    if (!is_console)
        uartusbinit(unit);

    return 1;
}

struct driver uartusbdriver = {
    "uartusb",
    uartusbprobe,
};
