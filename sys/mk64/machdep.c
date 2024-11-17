/*
 * Copyright (c) 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */
#include <sys/param.h>
#include <sys/dir.h>
#include <sys/inode.h>
#include <sys/user.h>
#include <sys/proc.h>
#include <sys/fs.h>
#include <sys/map.h>
#include <sys/buf.h>
#include <sys/file.h>
#include <sys/clist.h>
#include <sys/callout.h>
#include <sys/reboot.h>
#include <sys/msgbuf.h>
#include <sys/namei.h>
#include <sys/mount.h>
#include <sys/systm.h>
#include <sys/kconfig.h>
#include <sys/tty.h>

#include <machine/fault.h>
#include <machine/uart.h>
#include <machine/systick.h>

#if defined(TEENSY_LED_KERNEL)
#    define LED_KERNEL_INIT() teensy_led_init()
#    define LED_KERNEL_ON() teensy_led_on()
#    define LED_KERNEL_OFF() teensy_led_off()
#else
#    define LED_KERNEL_INIT() /* Nothing. */
#    define LED_KERNEL_ON()   /* Nothing. */
#    define LED_KERNEL_OFF()  /* Nothing. */
#endif

char machine[]      = MACHINE;      /* from <machine/machparam.h> */
char machine_arch[] = MACHINE_ARCH; /* from <machine/machparam.h> */
char cpu_model[64];

int hz     = HZ;
int usechz = (1000000L + HZ - 1) / HZ;
#ifdef TIMEZONE
struct timezone tz = {TIMEZONE, DST};
#else
struct timezone tz = {8 * 60, 1};
#endif
int nproc = NPROC;

struct namecache namecache[NNAMECACHE];
char bufdata[NBUF * MAXBSIZE];
struct inode inode[NINODE];
struct callout callout[NCALL];
struct mount mount[NMOUNT];
struct buf buf[NBUF], bfreelist[BQUEUES];
struct bufhd bufhash[BUFHSZ];
struct cblock cfree[NCLIST];
struct proc proc[NPROC];
struct file file[NFILE];

/*
 * Remove the ifdef/endif to run the kernel in unsecure mode even when in
 * a multiuser state.  Normally 'init' raises the security level to 1
 * upon transitioning to multiuser.  Setting the securelevel to -1 prevents
 * the secure level from being raised by init.
 */
#ifdef PERMANENTLY_INSECURE
int securelevel = -1;
#else
int securelevel = 0;
#endif

struct mapent swapent[SMAPSIZ];
struct map swapmap[1] = {
    {swapent, &swapent[SMAPSIZ], "swapmap"},
};

int waittime = -1;

static int nodump(dev)
dev_t dev;
{
    printf("\ndumping to dev %o off %D: not implemented\n", dumpdev,
           dumplo);
    return 0;
}

int (*dump)(dev_t) = nodump;

dev_t pipedev;
daddr_t dumplo = (daddr_t) 1024;


/*
 * Machine dependent startup code
 */
void startup() {

    /* Enable all configurable fault handlers. */
    arm_enable_fault(MM_FAULT_ENABLE);
    arm_enable_fault(BF_FAULT_ENABLE);
    arm_enable_fault(UF_FAULT_ENABLE);

    /* Syscalls (via PendSV) have the lowest interrupt priority. */
    arm_intr_set_priority(PENDSV_IRQ, IPL_PENDSV);

    /* SVCall exceptions have the highest interrupt priority. */
    arm_intr_set_priority(SVCALL_IRQ, IPL_SVCALL);

    /* SysTick exceptions have the interrupt priority of IPL_CLOCK. */
    arm_intr_set_priority(SYSTICK_IRQ, IPL_SYSTICK);

    /*
     * Configure LED pins.
     */
    LED_KERNEL_INIT(); /* the only one Paul had money for. */

    LED_KERNEL_ON();

    LED_KERNEL_OFF();

    led_control(LED_ALL, 1);
    led_control(LED_ALL, 0);

    /*
     * Configure User Button.
     */
    BUTTON_USER_INIT();

    /*
     * Early setup for console devices.
     */
#if CONS_MAJOR == UART_MAJOR
    uartinit(CONS_MINOR);
#endif

    /*
     * When User button is pressed - boot to single user mode.
     */
    boothowto = 0;
    if (BUTTON_USER_PRESSED()) {
        boothowto |= RB_SINGLE;
    }
}

static void cpuidentify() {
    /**
     * TODO: actually implement this
     */
    printf("cpu: ");

    physmem = 256 * 1024 - 8; /* yes, minus the eight bytes. */
    copystr("MK64FX512", cpu_model, sizeof(cpu_model), NULL);
    printf("MK64FX512");
    printf(", %u MHz, bus %u MHz\n", CPU_KHZ / 1000, BUS_KHZ / 1000);

    printf("oscillator: ");
    printf("unknown\n");
}

/*
 * Check whether the controller has been successfully initialized.
 */
static int is_controller_alive(driver, unit)
struct driver *driver;
int unit;
{
    struct conf_ctlr *ctlr;

    /* No controller - that's OK. */
    if (driver == 0)
        return 1;

    for (ctlr = conf_ctlr_init; ctlr->ctlr_driver; ctlr++) {
        if (ctlr->ctlr_driver == driver && ctlr->ctlr_unit == unit &&
            ctlr->ctlr_alive) {
            return 1;
        }
    }
    return 0;
}

/*
 * Configure all controllers and devices as specified
 * in the kernel configuration file.
 */
void kconfig() {
    struct conf_ctlr *ctlr;
    struct conf_device *dev;

    cpuidentify();

    /* Probe and initialize controllers first. */
    for (ctlr = conf_ctlr_init; ctlr->ctlr_driver; ctlr++) {
        if ((*ctlr->ctlr_driver->d_init)(ctlr)) {
            ctlr->ctlr_alive = 1;
        }
    }

    /* Probe and initialize devices. */
    for (dev = conf_device_init; dev->dev_driver; dev++) {
        if (is_controller_alive(dev->dev_cdriver, dev->dev_ctlr)) {
            if ((*dev->dev_driver->d_init)(dev)) {
                dev->dev_alive = 1;
            }
        }
    }
}

/*
 * Sit and wait for something to happen...
 */
void idle() {
    /* Indicate that no process is running. */
    noproc = 1;

    /* Set SPL low so we can be interrupted. */
    int x  = spl0();

    led_control(LED_KERNEL, 0);

    /* Wait for something to happen. */
    dsb();
    isb();
    wfi();

    /* Restore previous SPL. */
    splx(x);
}

void boot(dev, howto) register dev_t dev;
register int howto;
{
    if ((howto & RB_NOSYNC) == 0 && waittime < 0 &&
        bfreelist[0].b_forw) {
        register struct fs *fp;
        register struct buf *bp;
        int iter, nbusy;

        /*
         * Force the root filesystem's superblock to be updated,
         * so the date will be as current as possible after
         * rebooting.
         */
        fp = getfs(rootdev);
        if (fp)
            fp->fs_fmod = 1;
        waittime = 0;
        printf("syncing disks... ");
        (void) splnet();
        sync();
        for (iter = 0; iter < 20; iter++) {
            nbusy = 0;
            for (bp = &buf[NBUF]; --bp >= buf;)
                if (bp->b_flags & B_BUSY)
                    nbusy++;
            if (nbusy == 0)
                break;
            printf("%d ", nbusy);
            mdelay(40L * iter);
        }
        printf("done\n");
    }
    (void) splhigh();
    if (!(howto & RB_HALT)) {
        if ((howto & RB_DUMP) && dumpdev != NODEV) {
            /*
             * Take a dump of memory by calling (*dump)(),
             * which must correspond to dumpdev.
             * It should dump from dumplo blocks to the end
             * of memory or to the end of the logical device.
             */
            (*dump)(dumpdev);
        }
        /* Restart from dev, howto */

        /* Reset microcontroller */
        WRITE_RESTART(0x5FA0004);
        /* NOTREACHED */
    }
    printf("halted\n");

#ifdef HALTREBOOT
    printf("press any key to reboot...\n");
    cngetc();

    /* Reset microcontroller */
    WRITE_RESTART(0x5FA0004);
    /* NOTREACHED */
#endif

    printf("reboot failed; spinning\n");
    for (;;) {
        dsb();
        isb();
        wfi();
    }
    /* NOTREACHED */
}

/*
 * Millisecond delay routine.
 *
 * Uses SysTick, which must be configured to a 1ms timebase.
 * This is a busy-wait blocking delay, so be wise with use.
 */
/**
 * replaced with Teensy implementation. same disclaimer applies.
 */

unsigned int micros(void) {
    unsigned int count, current, istatus;

    __disable_irq();
    current = SYST_CVR;
    count   = systick_ms;
    istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
    __enable_irq();
    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50)
        count++;
    current = ((F_CPU / 1000) - 1) - current;
#if defined(KINETISL) && F_CPU == 48000000
    return count * 1000 + ((current * (unsigned int) 87381) >> 22);
#elif defined(KINETISL) && F_CPU == 24000000
    return count * 1000 + ((current * (unsigned int) 174763) >> 22);
#endif
    return count * 1000 + current / (F_CPU / 1000000);
}

void mdelay(unsigned int msec) {
    unsigned int start = micros();

    if (msec > 0) {
        while (1) {
            while ((micros() - start) >= 1000) {
                msec--;
                if (msec == 0)
                    return;
                start += 1000;
            }
        }
    }
}


/*
 * Control LEDs, installed on the board.
 */
void led_control(int mask, int on) {
    if (mask & LED_TTY) { /* Terminal i/o */
        if (on)
            LED_TTY_ON();
        else
            LED_TTY_OFF();
    }
    if (mask & LED_SWAP) { /* Auxiliary swap */
        if (on)
            LED_SWAP_ON();
        else
            LED_SWAP_OFF();
    }
    if (mask & LED_DISK) { /* Disk i/o */
        if (on)
            LED_DISK_ON();
        else
            LED_DISK_OFF();
    }
    if (mask & LED_KERNEL) { /* Kernel activity */
        if (on)
            LED_KERNEL_ON();
        else
            LED_KERNEL_OFF();
    }
}

/*
 * Increment user profiling counters.
 */
void addupc(caddr_t pc, struct uprof *pbuf, int ticks) {
    unsigned indx;

    if (pc < (caddr_t) pbuf->pr_off)
        return;

    indx = pc - (caddr_t) pbuf->pr_off;
    indx = (indx * pbuf->pr_scale) >> 16;
    if (indx >= pbuf->pr_size)
        return;

    pbuf->pr_base[indx] += ticks;
}

/*
 * ffs -- vax ffs instruction
 */
int ffs(mask)
register u_long mask;
{
    register int cnt;

    if (mask == 0)
        return (0);
    for (cnt = 1; !(mask & 1); cnt++)
        mask >>= 1;
    return (cnt);
}

/*
 * Copy a null terminated string from one point to another.
 * Returns zero on success, ENOENT if maxlength exceeded.
 * If lencopied is non-zero, *lencopied gets the length of the copy
 * (including the null terminating byte).
 */
int copystr(src, dest, maxlength, lencopied)
register caddr_t src, dest;
register u_int maxlength, *lencopied;
{
    caddr_t dest0 = dest;
    int error     = ENOENT;

    if (maxlength != 0) {
        while ((*dest++ = *src++) != '\0') {
            if (--maxlength == 0) {
                /* Failed. */
                goto done;
            }
        }
        /* Succeeded. */
        error = 0;
    }
done:
    if (lencopied != 0)
        *lencopied = dest - dest0;
    return error;
}

/*
 * Calculate the length of a string.
 */
size_t strlen(s)
register const char *s;
{
    const char *s0 = s;

    while (*s++ != '\0')
        ;
    return s - s0 - 1;
}

/*
 * Return 0 if a user address is valid.
 * There is only one memory region allowed for user: RAM.
 */
int baduaddr(addr)
register caddr_t addr;
{
    if (addr >= (caddr_t) __user_data_start &&
        addr < (caddr_t) __user_data_end)
        return 0;
    return 1;
}

/*
 * Return 0 if a kernel address is valid.
 * There are two memory regions allowed for kernel: RAM and flash.
 */
int badkaddr(addr)
register caddr_t addr;
{
    if (addr >= (caddr_t) __kernel_data_start &&
        addr < (caddr_t) __kernel_data_end)
        return 0;
    if (addr >= (caddr_t) __kernel_flash_start &&
        addr < (caddr_t) __kernel_flash_end)
        return 0;
    return 1;
}

/*
 * Insert the specified element into a queue immediately after
 * the specified predecessor element.
 */
void insque(void *element, void *predecessor) {
    struct que {
        struct que *q_next;
        struct que *q_prev;
    };
    register struct que *e    = (struct que *) element;
    register struct que *prev = (struct que *) predecessor;

    e->q_prev                 = prev;
    e->q_next                 = prev->q_next;
    prev->q_next->q_prev      = e;
    prev->q_next              = e;
}

/*
 * Remove the specified element from the queue.
 */
void remque(void *element) {
    struct que {
        struct que *q_next;
        struct que *q_prev;
    };
    register struct que *e = (struct que *) element;

    e->q_prev->q_next      = e->q_next;
    e->q_next->q_prev      = e->q_prev;
}

/*
 * Compare strings.
 */
int strncmp(const char *s1, const char *s2, size_t n) {
    register int ret, tmp;

    if (n == 0)
        return 0;
    do {
        ret = *s1++ - (tmp = *s2++);
    } while ((ret == 0) && (tmp != 0) && --n);
    return ret;
}

/* Nonzero if pointer is not aligned on a "sz" boundary.  */
#define UNALIGNED(p, sz) ((unsigned) (p) & ((sz) - 1))

/*
 * Copy data from the memory region pointed to by src0 to the memory
 * region pointed to by dst0.
 * If the regions overlap, the behavior is undefined.
 */
void bcopy(const void *src0, void *dst0, size_t nbytes) {
    unsigned char *dst       = dst0;
    const unsigned char *src = src0;
    unsigned *aligned_dst;
    const unsigned *aligned_src;

    //printf("bcopy (%08x, %08x, %d)\n", src0, dst0, nbytes);
    /* If the size is small, or either SRC or DST is unaligned,
     * then punt into the byte copy loop.  This should be rare.  */
    if (nbytes >= 4 * sizeof(unsigned) &&
        !UNALIGNED(src, sizeof(unsigned)) &&
        !UNALIGNED(dst, sizeof(unsigned))) {
        aligned_dst = (unsigned *) dst;
        aligned_src = (const unsigned *) src;

        /* Copy 4X unsigned words at a time if possible.  */
        while (nbytes >= 4 * sizeof(unsigned)) {
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            nbytes -= 4 * sizeof(unsigned);
        }

        /* Copy one unsigned word at a time if possible.  */
        while (nbytes >= sizeof(unsigned)) {
            *aligned_dst++ = *aligned_src++;
            nbytes -= sizeof(unsigned);
        }

        /* Pick up any residual with a byte copier.  */
        dst = (unsigned char *) aligned_dst;
        src = (const unsigned char *) aligned_src;
    }

    while (nbytes--)
        *dst++ = *src++;
}

void *memcpy(void *dst, const void *src, size_t nbytes) {
    bcopy(src, dst, nbytes);
    return dst;
}

/*
 * Fill the array with zeroes.
 */
void bzero(void *dst0, size_t nbytes) {
    unsigned char *dst;
    unsigned *aligned_dst;

    dst = (unsigned char *) dst0;
    while (UNALIGNED(dst, sizeof(unsigned))) {
        *dst++ = 0;
        if (--nbytes == 0)
            return;
    }
    if (nbytes >= sizeof(unsigned)) {
        /* If we get this far, we know that nbytes is large and dst is word-aligned. */
        aligned_dst = (unsigned *) dst;

        while (nbytes >= 4 * sizeof(unsigned)) {
            *aligned_dst++ = 0;
            *aligned_dst++ = 0;
            *aligned_dst++ = 0;
            *aligned_dst++ = 0;
            nbytes -= 4 * sizeof(unsigned);
        }
        while (nbytes >= sizeof(unsigned)) {
            *aligned_dst++ = 0;
            nbytes -= sizeof(unsigned);
        }
        dst = (unsigned char *) aligned_dst;
    }

    /* Pick up the remainder with a bytewise loop.  */
    while (nbytes--)
        *dst++ = 0;
}

/*
 * Compare not more than nbytes of data pointed to by m1 with
 * the data pointed to by m2. Return an integer greater than, equal to or
 * less than zero according to whether the object pointed to by
 * m1 is greater than, equal to or less than the object
 * pointed to by m2.
 */
int bcmp(const void *m1, const void *m2, size_t nbytes) {
    const unsigned char *s1 = (const unsigned char *) m1;
    const unsigned char *s2 = (const unsigned char *) m2;
    const unsigned *aligned1, *aligned2;

    /* If the size is too small, or either pointer is unaligned,
     * then we punt to the byte compare loop.  Hopefully this will
     * not turn up in inner loops.  */
    if (nbytes >= 4 * sizeof(unsigned) &&
        !UNALIGNED(s1, sizeof(unsigned)) &&
        !UNALIGNED(s2, sizeof(unsigned))) {
        /* Otherwise, load and compare the blocks of memory one
           word at a time.  */
        aligned1 = (const unsigned *) s1;
        aligned2 = (const unsigned *) s2;
        while (nbytes >= sizeof(unsigned)) {
            if (*aligned1 != *aligned2)
                break;
            aligned1++;
            aligned2++;
            nbytes -= sizeof(unsigned);
        }

        /* check remaining characters */
        s1 = (const unsigned char *) aligned1;
        s2 = (const unsigned char *) aligned2;
    }
    while (nbytes--) {
        if (*s1 != *s2)
            return *s1 - *s2;
        s1++;
        s2++;
    }
    return 0;
}

int copyout(caddr_t from, caddr_t to, u_int nbytes) {
    //printf("copyout (from=%p, to=%p, nbytes=%u)\n", from, to, nbytes);
    if (baduaddr(to) || baduaddr(to + nbytes - 1))
        return EFAULT;
    bcopy(from, to, nbytes);
    return 0;
}

int copyin(caddr_t from, caddr_t to, u_int nbytes) {
    if (baduaddr(from) || baduaddr(from + nbytes - 1))
        return EFAULT;
    bcopy(from, to, nbytes);
    return 0;
}
