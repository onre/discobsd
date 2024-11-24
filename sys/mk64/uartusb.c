/*
 * Teensy 3.5 virtual serial over USB. 
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
#include <machine/machparam.h>
#include <machine/teensy_usb_dev.h>
#include <machine/teensy_usb_serial.h>

#define EOF (-1)

struct tty uartusbttys[1];

void uartusbstart(struct tty *tp);

struct uartusb_inst {
    volatile u_char rx_buffer[RX_BUFFER_SIZE];
    volatile u_char tx_buffer[TX_BUFFER_SIZE];
    volatile u_char rx_buffer_head;
    volatile u_char rx_buffer_tail;
    volatile u_char tx_buffer_head;
    volatile u_char tx_buffer_tail;
};

static struct uartusb_inst uartusb[1] = {{{0}, {0}, 0, 0, 0, 0}};

/**
 * ...I know.
 */
void ftm0_isr(void) {
    int s;
    s = spltty();

    usb_isr();

    splx(s);
}

void uartusbinit(int unit) {
    register struct uartusb_inst *inst; 
    
    if (unit != 0)
        return;

    inst = &uartusb[unit];
    if (unit == 0)
	usb_serial_set_callback(uartusbintr);
}

int uartusbopen(dev_t dev, int flag, int mode) {
    register struct uartusb_inst *uip; 
    register struct tty *tp;
    register int unit = minor(dev);

    if (unit != 0)
        return (ENXIO);

    tp          = &uartusbttys[unit];

    uip         = (struct uartusb_inst *) tp->t_addr;
    tp->t_oproc = uartusbstart;

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

    if (tp->t_ispeed == 0) {
	tp->t_ispeed = 115200;
	tp->t_ospeed = 115200;
    }

    return ttyopen(dev, tp);
}

int uartusbclose(dev_t dev, int flag, int mode) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartusbttys[unit];

    ttywflush(tp);
    ttyclose(tp);
    return (0);
}

int uartusbread(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartusbttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return ttread(tp, uio, flag);
}

int uartusbwrite(dev_t dev, struct uio *uio, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartusbttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return ttwrite(tp, uio, flag);
}

int uartusbselect(dev_t dev, int rw) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartusbttys[unit];

    if (!tp->t_addr)
        return ENODEV;

    return (ttyselect(tp, rw));
}

int uartusbioctl(dev_t dev, u_int cmd, caddr_t addr, int flag) {
    register int unit       = minor(dev);
    register struct tty *tp = &uartusbttys[unit];
    register int error;

    if (!tp->t_addr)
        return ENODEV;

    error = ttioctl(tp, cmd, addr, flag);
    if (error < 0)
        error = ENOTTY;
    return (error);
}

void uartusbintr(dev_t dev) {
    register int c, s;
    register int unit       = minor(dev);
    register struct tty *tp;
    register struct uartusb_inst *uip;

    s = spltty();

    if (unit != 0)
	goto out;

    tp          = &uartusbttys[unit];

    if (!tp->t_addr) {
        goto out;
    }
    
    /**
     * should usb_isr() call this when it has data? or what? how?
     * should this call usb_isr()? what? why?
     */
    
    uip         = (struct uartusb_inst *) tp->t_addr;

    if (tp->t_outq.c_cc) {
        while (tp->t_outq.c_cc) {
	    c = getc(&tp->t_outq);
	    usb_serial_putchar(c);
        }

	if (tp->t_state & TS_BUSY) {
	    tp->t_state &= ~TS_BUSY;
	    ttstart(tp);
	}
    }
 out:
    splx(s);
}

/*
 * turn off 
 */
void uartusbstart(struct tty *tp) {
    register struct uartusb_inst *uip;
    register int c, s;
    
    s = spltty();
    
    if (!tp->t_addr)
        return;

    uip         = (struct uartusb_inst *) tp->t_addr;

    /**
     * terminal has got other things to do, so let's just bail
     */
    if (tp->t_state & (TS_TIMEOUT | TS_BUSY | TS_TTSTOP)) {
    out:
        splx(s);
        return;
    }
    ttyowake(tp);
    if (tp->t_outq.c_cc == 0) /* nothing in queue, so bail out */
        goto out;

    while (tp->t_outq.c_cc) {
        c = getc(&tp->t_outq);
	usb_serial_putchar(c);
    };

    tp->t_state &= ~TS_BUSY;

    splx(s);
}

void uartusbputc(dev_t dev, char c) {
    int unit       = minor(dev);
    struct tty *tp = &uartusbttys[unit];
    register int s;

    if (!tp->t_addr)
        return;

    s = spltty();

    if (usb_configuration) {
        usb_serial_putchar(c);
    }

    splx(s);
}

char uartusbgetc(dev_t dev) {
    int unit = minor(dev);
    int s, c;

    s = spltty();

    if (!usb_configuration) {
        c = EOF;
	goto out;
    }

    for (;;) {
        /* Wait for key pressed. */
        if (usb_serial_available()) {
            c = usb_serial_getchar();
            break;
        }
    }

 out:
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

    printf("uartusb%d: enabled", unit);

    if (is_console)
        printf(", console");
    printf("\n");

    /* Initialize the device. */
    uartusbttys[unit].t_addr = (caddr_t) &uartusb[unit];
    if (!is_console)
        uartusbinit(unit);

    return 1;
}

struct driver uartusbdriver = {
    "uartusb",
    uartusbprobe,
};
