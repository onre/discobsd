#
# Public Domain.  1996/11/16 - Steven Schultz
#
TOPSRC!=cd ../..; pwd
include ${TOPSRC}/share/mk/sys.mk

CFLAGS		+= -Wall

SRCS		= hurtkernel.c
OBJS		= hurtkernel.o

all:            hurtkernel

hurtkernel:         ${OBJS}
		${CC} ${CFLAGS} ${LDFLAGS} -o hurtkernel.elf ${OBJS} ${LIBS}
		${OBJDUMP} -S hurtkernel.elf > hurtkernel.dis
		${SIZE} hurtkernel.elf
		${ELF2AOUT} hurtkernel.elf $@ && rm hurtkernel.elf

clean:
		rm -f *.o *.elf hurtkernel *.elf *.dis tags *~

depend:         ${SRCS}
		mkdep ${CFLAGS} ${SRCS}

install:        hurtkernel
		${INSTALL} hurtkernel ${DESTDIR}/usr/bin/hurtkernel

lint:           ${SRCS}
		lint -hax ${SRCS}

tags:           ${SRCS}
		ctags ${SRCS}

# DO NOT DELETE THIS LINE -- mkdep uses it.
# DO NOT PUT ANYTHING AFTER THIS LINE, IT WILL GO AWAY.
