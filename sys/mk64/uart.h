#ifndef _UART_H
#define _UART_H

#include <sys/types.h>
#include <sys/tty.h>

#ifdef KERNEL

#undef UART_OVERFLOWSTATS

#define NUART   2

#define C2_ENABLE		UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE

#define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE		C2_ENABLE

/* for UART0, UART1 */
#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))

/* for the rest */
#define BAUD2DIV_BUS(baud) (((F_BUS * 2) + ((baud) >> 1)) / (baud))

#define SERIAL_7E1 0x02
#define SERIAL_7O1 0x03
#define SERIAL_8N1 0x00
#define SERIAL_8E1 0x06
#define SERIAL_8O1 0x07


void            uartinit(int unit);
int             uartopen(dev_t dev, int flag, int mode);
int             uartclose(dev_t dev, int flag, int mode);
int             uartread(dev_t dev, struct uio *uio, int flag);
int             uartwrite(dev_t dev, struct uio *uio, int flag);
int             uartselect(dev_t dev, int rw);
int             uartioctl(dev_t dev, u_int cmd, caddr_t addr, int flag);
void            uartintr(dev_t dev);
void            uartstart(struct tty *tp);
void            uartputc(dev_t dev, char c);
char            uartgetc(dev_t dev);

extern struct   tty uartttys[NUART];

#endif /* KERNEL */

#endif /* !_UART_H */
