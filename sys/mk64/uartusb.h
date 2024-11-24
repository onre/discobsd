#ifndef _UARTUSB_H
#define _UARTUSB_H

#ifdef KERNEL

#include <sys/types.h>
#include <sys/tty.h>

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

void            ftm0_isr(void);

void            uartusbinit(int unit);
int             uartusbopen(dev_t dev, int flag, int mode);
int             uartusbclose(dev_t dev, int flag, int mode);
int             uartusbread(dev_t dev, struct uio *uio, int flag);
int             uartusbwrite(dev_t dev, struct uio *uio, int flag);
int             uartusbselect(dev_t dev, int rw);
int             uartusbioctl(dev_t dev, u_int cmd, caddr_t addr, int flag);
void            uartusbintr(dev_t dev);
void            uartusbstart(struct tty *tp);
void            uartusbputc(dev_t dev, char c);
char            uartusbgetc(dev_t dev);

extern struct   tty uartusbttys[1];

#endif /* KERNEL */

#endif /* !_UARTUSB_H */
