#ifndef _USB_UART_H
#define _USB_UART_H

#ifdef KERNEL

#include <sys/types.h>
#include <sys/tty.h>

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

void            uartusbinit(int unit);
int             usbuartopen(dev_t dev, int flag, int mode);
int             usbuartclose(dev_t dev, int flag, int mode);
int             usbuartread(dev_t dev, struct uio *uio, int flag);
int             usbuartwrite(dev_t dev, struct uio *uio, int flag);
int             usbuartselect(dev_t dev, int rw);
int             usbuartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag);
void            usbuartintr(dev_t dev);
void            usbuartstart(struct tty *tp);
void            usbuartputc(dev_t dev, char c);
char            usbuartgetc(dev_t dev);

extern struct   tty usbuartttys[1];

#endif /* KERNEL */

#endif /* !_USB_UART_H */
