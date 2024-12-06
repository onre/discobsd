/*
 * Machine dependent constants for Kinetis
 *
 * Copyright (c) 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */

#include <sys/types.h>

#if defined(USB_DISABLED) && defined(USB_SERIAL)
#error can not define USB_DISABLED and USB_SERIAL simultaneously
#endif

/*
 * SIMPLE_INTERRUPTS - preset all interrupts to equal priority in startup code and then
 * make sure none of the kernel code changes any of those priorities.
 *
 * TODO: implement.
 */

#ifdef SIMPLE_INTERRUPTS

#define SET_DEFAULT_INTERRUPT_PRIORITY
#define SPL_DEFAULT 128

#define SPL_LEAST     SPL_DEFAULT
#define SPL_SOFTCLOCK SPL_DEFAULT
#define SPL_NET       SPL_DEFAULT
#define SPL_TTY       SPL_DEFAULT
#define SPL_BIO       SPL_DEFAULT
#define SPL_CLOCK     SPL_DEFAULT
#define SPL_HIGH      SPL_DEFAULT
#define SPL_TOP       SPL_DEFAULT

#else

#define HARDMODE

#define SET_DEFAULT_INTERRUPT_PRIORITY
#define SPL_DEFAULT SPL_LEAST

#endif


#ifndef ENDIAN

#define MACHINE      "mk64"
#define MACHINE_ARCH "arm"

/*
 * Definitions for byte order,
 * according to byte significance from low address to high.
 */
#define LITTLE       1234 /* least-significant byte first (vax) */
#define BIG          4321 /* most-significant byte first */
#define PDP          3412 /* LSB first in word, MSW first in long (pdp) */
#define ENDIAN       LITTLE /* byte order on stm32 */

/*
 * The time for a process to be blocked before being very swappable.
 * This is a number of seconds which the system takes as being a non-trivial
 * amount of real time.  You probably shouldn't change this;
 * it is used in subtle ways (fractions and multiples of it are, that is, like
 * half of a ``long time'', almost a long time, etc.)
 * It is related to human patience and other factors which don't really
 * change over time.
 */
#define MAXSLP       20

/*
 * Clock ticks per second. The HZ value must be an integer factor of 1000.
 * Cortex-M SysTick operates with a 1ms time base, hence 1000 for HZ.
 */
#ifndef HZ
#define HZ 1000
#endif

/*
 * System parameter formulae.
 */
#ifndef NBUF
#define NBUF 10 /* number of i/o buffers */
#endif
#ifndef MAXUSERS
#define MAXUSERS 1 /* number of user logins */
#endif
#ifndef NPROC
#define NPROC 25 /* number of processes */
#endif
#ifndef NINODE
#define NINODE 24
#endif
#ifndef NFILE
#define NFILE 24
#endif
#define NNAMECACHE (NINODE * 11 / 10)
#define NCALL      (16 + 2 * MAXUSERS)
#define NCLIST     32 /* number or CBSIZE blocks */
#ifndef SMAPSIZ
#define SMAPSIZ NPROC /* size of swap allocation map */
#endif

/*
 * Disk blocks.
 */
#define DEV_BSIZE  1024 /* the same as MAXBSIZE */
#define DEV_BSHIFT 10   /* log2(DEV_BSIZE) */
#define DEV_BMASK  (DEV_BSIZE - 1)

/* Bytes to disk blocks */
#define btod(x)    (((x) + DEV_BSIZE - 1) >> DEV_BSHIFT)

#if 1 /* XXX Needed for ps, w, smlrc. To be removed. */
#define USER_DATA_START (0x20000000)
#define USER_DATA_SIZE                                                 \
    191 * 1024 /* almost 192 kb (196600 bytes) of user RAM. */
#define USER_DATA_END (USER_DATA_START + USER_DATA_SIZE)

#define stacktop(siz) (USER_DATA_END)
#define stackbas(siz) (USER_DATA_END - (siz))
#endif /* XXX Needed for ps, w, smlrc. To be removed. */

/*
 * User area: a user structure, followed by the kernel
 * stack.  The number for USIZE is determined empirically.
 *
 * Note that the SBASE and STOP constants are only used by the assembly code,
 * but are defined here to localize information about the user area's
 * layout (see pdp/genassym.c).  Note also that a networking stack is always
 * allocated even for non-networking systems.  This prevents problems with
 * applications having to be recompiled for networking versus non-networking
 * systems.
 */
#define USIZE  3072
#define SSIZE  2048 /* initial stack size (bytes) */

#define MAXMEM (191 * 1024)

/*
 * Collect kernel statistics by default.
 */
#if !defined(UCB_METER) && !defined(NO_UCB_METER)
#define UCB_METER
#endif

#ifdef KERNEL
#include <machine/intr.h>

/*
 * Macros to decode processor status word.
 */
#define USERMODE(psr) ((psr & 0x1FFUL) == 0)
#define BASEPRI(psr)  get_basepri()

#define noop()        asm volatile("nop")

/*
 * Wait for something to happen.
 */
void idle(void);

/*
 * Millisecond delay routine.
 */
void mdelay(unsigned msec);

/*
 * Setup system timer for `hz' timer interrupts per second.
 */
void clkstart(void);

/*
 * the memcpy function from memcpy-armv7m.S
 */
void *memcpy(void *dst, const void *src, size_t count);

/*
 * memset from memset.S
 */
void *memset(void *s, int c, size_t n);

/*
 * Control LEDs, installed on the board.
 */
#define LED_TTY    0x08
#define LED_SWAP   0x04
#define LED_DISK   0x02
#define LED_KERNEL 0x01
#define LED_ALL    (LED_TTY | LED_SWAP | LED_DISK | LED_KERNEL)

void led_control(int mask, int on);

/*
 * SD timeouts, for sysctl.
 */
extern int sd_timo_cmd;
extern int sd_timo_send_op;
extern int sd_timo_send_csd;
extern int sd_timo_read;
extern int sd_timo_wait_cmd;
extern int sd_timo_wait_wdata;
extern int sd_timo_wait_wdone;
extern int sd_timo_wait_wstop;
extern int sd_timo_wait_widle;

#endif /* KERNEL */

#endif /* ENDIAN */
