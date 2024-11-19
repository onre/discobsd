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

#include <machine/usb_uart.h>
#include <machine/teensy_usb_serial.h>

struct tty usbuartttys[1];

void cnstart(struct tty *tp);

/**
 *
 * the only instance has already been initialized by the startup code,
 * so the routines are devoid of action.
 *
 */

void usbuartinit(int unit) {
    if (unit != 0)
        return;

    usbuartttys[0].t_addr =
        (caddr_t) 2588; /* we need something so that the check does not fail */
}

int usbuartopen(dev_t dev, int flag, int mode) {
    register struct tty *tp;
    register int unit = minor(dev);

    if (unit != 0)
        return (ENXIO);

    tp          = &usbuartttys[0];

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
    register struct tty *tp = &usbuartttys[0];

    if (!tp->t_addr)
        return ENODEV;

    ttywflush(tp);
    ttyclose(tp);
    return (0);
}

/*ARGSUSED*/
int usbuartread(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[0];

    if (!tp->t_addr)
        return ENODEV;

    return ttread(tp, uio, flag);
}

/*ARGSUSED*/
int usbuartwrite(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[0];

    if (!tp->t_addr)
        return ENODEV;

    return ttwrite(tp, uio, flag);
}

int usbuartselect(dev_t dev, int rw) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[0];

    if (!tp->t_addr)
        return ENODEV;

    return (ttyselect(tp, rw));
}

/*ARGSUSED*/
int usbuartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[0];
    register int error;

    if (!tp->t_addr)
        return ENODEV;

    error = ttioctl(tp, cmd, addr, flag);
    if (error < 0)
        error = ENOTTY;
    return (error);
}

void usbuartintr(dev_t dev) {
    register int c;
    register int unit       = minor(dev);
    register struct tty *tp = &usbuartttys[0];

    if (!tp->t_addr)
        return;

    /* Receive */
    while (usb_serial_available()) {
        c = usb_serial_getchar();
        ttyinput(c, tp);
    }

#if 0
    if (LL_USART_IsActiveFlag_TXE(uip->inst)) {
        led_control(LED_TTY, 0);

        /* Disable transmit interrupt. */
        LL_USART_DisableIT_TXE(uip->inst);
#endif

    if (tp->t_state & TS_BUSY) {
        tp->t_state &= ~TS_BUSY;
        ttstart(tp);
    }
#if 0 
    }
#endif
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

    c = getc(&tp->t_outq);
    usb_serial_putchar(c);
    tp->t_state |= TS_BUSY;

    led_control(LED_TTY, 1);
    splx(s);
}

void usbuartputc(dev_t dev, char c) {
    int unit       = minor(dev);
    struct tty *tp = &usbuartttys[0];
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
static int usbuartprobe(struct conf_device *config) {
    int unit       = config->dev_unit - 1;
    int is_console = (CONS_MAJOR == UART_MAJOR && CONS_MINOR == unit);

    if (unit != 0)
        return 0;

    printf("uart%d: virtual over USB");

    if (is_console)
        printf(", console");
    printf("\n");

    /* Initialize the device. */
    usbuartttys[0].t_addr = (caddr_t) 2588; /* sorry */
    if (!is_console)
        usbuartinit(unit);

    return 1;
}

struct driver usbuartdriver = {
    "usbuart",
    usbuartprobe,
};
