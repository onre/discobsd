/*
 * Copyright (c) 2022 Christopher Hettrick <chris@structfoo.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/signalvar.h>
#include <sys/systm.h>
#include <sys/user.h>
#include <sys/proc.h>
#include <sys/vm.h>

#include <machine/frame.h>
#include <machine/intr.h>
#include <machine/debug.h>

#ifdef DEBUG_PRINT_SYSCALLS
u_char scstrarg[] = {
    0,            /*   0 = indir */
    0,             /*   1 = exit */
    0,             /*   2 = fork */
    0,             /*   3 = read */
    0,            /*   4 = write */
    1,             /*   5 = open */
    0,            /*   6 = close */
    0,            /*   7 = wait4 */
    0,               /*   8 = (old creat) */
    3,             /*   9 = link */
    1,           /*  10 = unlink */
    1,            /*  11 = execv */
    1,            /*  12 = chdir */
    0,           /*  13 = fchdir */
    1,            /*  14 = mknod */
    1,            /*  15 = chmod */
    1,            /*  16 = chown; now 3 args */
    1,          /*  17 = chflags */
    0,         /*  18 = fchflags */
    0,            /*  19 = lseek */
    0,           /*  20 = getpid */
    3,            /*  21 = mount */
    0,           /*  22 = umount */
    0,         /*  23 = __sysctl */
    0,           /*  24 = getuid */
    0,          /*  25 = geteuid */
    0,           /*  26 = ptrace */
    0,          /*  27 = getppid */
    1,           /*  28 = statfs */
    0,          /*  29 = fstatfs */
    0,        /*  30 = getfsstat */
    0,        /*  31 = sigaction */
    0,      /*  32 = sigprocmask */
    1,           /*  33 = access */
    0,       /*  34 = sigpending */
    0,      /*  35 = sigaltstack */
    0,             /*  36 = sync */
    0,             /*  37 = kill */
    1,             /*  38 = stat */
    0,         /*  39 = getlogin */
    1,            /*  40 = lstat */
    0,              /*  41 = dup */
    0,             /*  42 = pipe */
    1,         /*  43 = setlogin */
    0,           /*  44 = profil */
    0,           /*  45 = setuid */
    0,          /*  46 = seteuid */
    0,           /*  47 = getgid */
    0,          /*  48 = getegid */
    0,           /*  49 = setgid */
    0,          /*  50 = setegid */
    0,          /*  51 = kmemdev  */
    0,             /*  52 = (2.9) set phys addr */
    0,             /*  53 = (2.9) lock in core */
    0,            /*  54 = ioctl */
    0,           /*  55 = reboot */
    0,          /*  56 = sigwait */
    3,          /*  57 = symlink */
    1,         /*  58 = readlink */
    1,           /*  59 = execve */
    0,            /*  60 = umask */
    1,           /*  61 = chroot */
    0,            /*  62 = fstat */
    0,              /*  63 = unused */
    0,              /*  64 = (old getpagesize) */
    0,          /*  65 = pselect */
    0,            /*  66 = vfork */
    0,              /*  67 = unused */
    0,              /*  68 = unused */
    0,              /*  69 = brk */
    0,           /*  70 = read from global space */
    0,           /*  71 = write to global space */
    0,             /*  72 = kticks */
    0,              /*  73 = unused */
    0,              /*  74 = unused */
    0,              /*  75 = unused */
    0,          /*  76 = vhangup */
    0,              /*  77 = unused */
    0,              /*  78 = unused */
    0,        /*  79 = getgroups */
    0,        /*  80 = setgroups */
    0,          /*  81 = getpgrp */
    0,          /*  82 = setpgrp */
    0,        /*  83 = setitimer */
    0,         /*  84 = wait,wait3 COMPAT*/
    0,              /*  85 = unused */
    0,        /*  86 = getitimer */
    0,              /*  87 = (old gethostname) */
    0,              /*  88 = (old sethostname) */
    0,    /*  89 = getdtablesize */
    0,             /*  90 = dup2 */
    0,              /*  91 = unused */
    0,            /*  92 = fcntl */
    0,           /*  93 = select */
    0,              /*  94 = unused */
    0,            /*  95 = fsync */
    0,      /*  96 = setpriority */
    0,           /*  97 = socket */
    0,          /*  98 = connect */
    0,           /*  99 = accept */
    0,      /* 100 = getpriority */
    0,             /* 101 = send */
    0,             /* 102 = recv */
    0,        /* 103 = sigreturn */
    0,             /* 104 = bind */
    0,       /* 105 = setsockopt */
    0,           /* 106 = listen */
    0,       /* 107 = sigsuspend */
    0,             /* 108 = (old sigvec) */
    0,             /* 109 = (old sigblock) */
    0,             /* 110 = (old sigsetmask) */
    0,             /* 111 = (old sigpause)  */
    0,         /* 112 = sigstack COMPAT-43 */
    0,          /* 113 = recvmsg */
    0,          /* 114 = sendmsg */
    0,             /* 115 = unused */
    0,     /* 116 = gettimeofday */
    0,        /* 117 = getrusage */
    0,       /* 118 = getsockopt */
    0,             /* 119 = unused */
    0,            /* 120 = readv */
    0,           /* 121 = writev */
    0,     /* 122 = settimeofday */
    0,           /* 123 = fchown */
    0,           /* 124 = fchmod */
    0,         /* 125 = recvfrom */
    0,             /* 126 = (old setreuid) */
    0,             /* 127 = (old setregid) */
    0,           /* 128 = rename */
    0,         /* 129 = truncate */
    0,        /* 130 = ftruncate */
    0,            /* 131 = flock */
    0,             /* 132 = unused */
    0,           /* 133 = sendto */
    0,         /* 134 = shutdown */
    0,       /* 135 = socketpair */
    0,            /* 136 = mkdir */
    0,            /* 137 = rmdir */
    0,           /* 138 = utimes */
    0,             /* 139 = unused */
    0,          /* 140 = adjtime */
    0,      /* 141 = getpeername */
    0,             /* 142 = (old gethostid) */
    0,             /* 143 = (old sethostid) */
    0,        /* 144 = getrlimit */
    0,        /* 145 = setrlimit */
    0,           /* 146 = killpg */
    0,             /* 147 = unused */
    0,         /* 148 = setquota */
    0,            /* 149 = quota */
    0,      /* 150 = getsockname */
    0,             /* 151 = unused */
    0,           /* 152 = ustore */
    0,           /* 153 = ufetch */
    0,            /* 154 = ucall */
    0             /* 155 = unused */
    
};

static void
print_args(int code, int narg, int arg0, int arg1, int arg2, int arg3,
           int arg4, int arg5);
static void print_arg(int val, int code, int n);

static void print_arg(int val, int code, int n) {
    if (code < 155 && (scstrarg[code] & (1 << n)))
	printf("\"%s\"", val);
    else if (val & 0xff000000)
	printf("%08x", val);
    else
	printf("%u", val);
}

static void
print_args(int code, int narg, int arg0, int arg1, int arg2, int arg3, int arg4, int arg5)
{
    print_arg(arg0, code, 0);
    if (narg > 1) { printf(", "); print_arg(arg1, code, 1); }
    if (narg > 2) { printf(", "); print_arg(arg2, code, 2); }
    if (narg > 3) { printf(", "); print_arg(arg3, code, 3); }
    if (narg > 4) { printf(", "); print_arg(arg4, code, 4); }
    if (narg > 5) { printf(", "); print_arg(arg5, code, 5); }
}
#endif

/**
 * had to make decisions, opted for Paul's naming.
 *
 *               mini-glossary
 *
 *         thread mode == user mode
 *        handler mode == supervisor mode
 *  
 */

/*
 * svcall_isr(frame)
 *	struct trapframe *frame;
 *
 * Exception handler entry point for system calls (via 'svc' instruction).
 * The real work is done in PendSV_Handler at the lowest exception priority.
 */
void svcall_isr(void) {
    /* Set a PendSV exception to immediately tail-chain into. */
    SCB_ICSR |= SCB_ICSR_PENDSVSET_MASK;

    __set_barrier();

    /* PendSV has lowest priority, so need to allow it to fire. */
    (void) spl0();
}

/*
 * pendablesrvreq_isr(frame)
 *	struct trapframe *frame;
 *
 * System call handler (via SVC_Handler pending a PendSV exception).
 * Save the processor state in a trap frame and pass it to syscall().
 * Restore processor state from returned trap frame on return from syscall().
 */
void pendablesrvreq_isr(void) {
    __asm volatile(
        "	.syntax	unified		\n\t"
        "	.thumb			\n\t"

        "	cpsid	i		\n\t" /* Disable interrupts. */

#ifdef __thumb2__
        /*
	 * ARMv7-M hardware already pushed r0-r3, ip, lr, pc, psr on PSP,
	 * and then switched to MSP and is currently in Handler Mode.
	 */
        "	push	{r4-r11}	\n\t" /* Push v1-v8 registers onto MSP. */
        "	mrs	r1, PSP		\n\t" /* Get pointer to trap frame. */
        "	ldmfd	r1, {r2-r9}	\n\t" /* Copy trap frame from PSP. */
        "	mov	r6, r1		\n\t" /* Set trap frame sp as PSP. */
        "	push	{r2-r9}		\n\t" /* Push that trap frame onto MSP. */

        "	mrs	r0, MSP		\n\t" /* MSP trap frame is syscall() arg. */
        "	bl	syscall		\n\t" /* Call syscall() with MSP as arg. */

        "	pop	{r2-r9}		\n\t" /* Pop off trap frame from MSP. */
        "	mov	r1, r6		\n\t" /* PSP will be trap frame sp. */
        "	stmia	r1, {r2-r9}	\n\t" /* Hardware pops off PSP on return. */
        "	msr	PSP, r1		\n\t" /* Set PSP as trap frame sp. */
        "	pop	{r4-r11}	\n\t" /* Pop from MSP into v1-v8 regs. */

        /*
	 * On return, ARMv7-M hardware sets PSP as stack pointer,
	 * pops from PSP to registers r0-r3, ip, lr, pc, psr,
	 * and then switches back to Thread Mode (exception completed).
	 */
        "	mov	lr, #0xFFFFFFFD	\n\t" /* EXC_RETURN Thread Mode, PSP */
    /* Return to Thread Mode. */
#else /* __thumb__ */
        /*
	 * ARMv6-M hardware already pushed r0-r3, ip, lr, pc, psr on PSP,
	 * and then switched to MSP and is currently in Handler Mode.
	 */
        "	mov	r0, r8		\n\t" /* Bring high register v5 to low. */
        "	mov	r1, r9		\n\t" /* Bring high register v6 to low. */
        "	mov	r2, r10		\n\t" /* Bring high register v7 to low. */
        "	mov	r3, r11		\n\t" /* Bring high register v8 to low. */
        "	push	{r0-r3}		\n\t" /* Push v5-v8 registers onto MSP. */
        "	push	{r4-r7}		\n\t" /* Push v1-v4 registers onto MSP. */

        "	mrs	r1, PSP		\n\t" /* Get pointer to trap frame. */
        "	mov	r2, r1		\n\t" /* Pointer to use for top half. */
        "	adds	r2, #(4 * 4)	\n\t" /* Index to top half of trap frame. */
        "	ldmfd	r2!, {r4-r7}	\n\t" /* Copy frame top half from PSP. */
        "	mov	r4, r1		\n\t" /* Set trap frame sp as PSP. */
        "	push	{r4-r7}		\n\t" /* Push frame top half onto MSP. */
        "	ldmfd	r1!, {r4-r7}	\n\t" /* Copy frame low half from PSP. */
        "	push	{r4-r7}		\n\t" /* Push frame low half onto MSP. */

        "	mrs	r0, MSP		\n\t" /* MSP trap frame is syscall() arg. */
        "	bl	syscall		\n\t" /* Call syscall() with MSP as arg. */

        "	pop	{r0-r7}		\n\t" /* Pop off trap frame from MSP. */
        "	msr	PSP, r4		\n\t" /* Set PSP as trap frame sp. */
        "	stmia	r4!, {r0-r3}	\n\t" /* Copy trap frame low half to PSP. */
        "	mrs	r1, PSP		\n\t" /* Get PSP again as trap frame sp. */
        "	stmia	r4!, {r1,r5-r7}	\n\t" /* Copy trap frame top half to PSP. */

        "	pop	{r4-r7}		\n\t" /* Pop from MSP into v1-v4 regs. */
        "	pop	{r0-r3}		\n\t" /* Pop from MSP for v5-v8 regs. */
        "	mov	r11, r3		\n\t" /* Move low register to high v8. */
        "	mov	r10, r2		\n\t" /* Move low register to high v7. */
        "	mov	r9, r1		\n\t" /* Move low register to high v6. */
        "	mov	r8, r0		\n\t" /* Move low register to high v5. */

        /*
	 * On return, ARMv6-M hardware sets PSP as stack pointer,
	 * pops from PSP to registers r0-r3, ip, lr, pc, psr,
	 * and then switches back to Thread Mode (exception completed).
	 */
        "	ldr	r1, =0xFFFFFFFD	\n\t" /* EXC_RETURN Thread Mode, PSP */
        "	mov	lr, r1		\n\t" /* Return to Thread Mode. */
#endif
    );
}

void syscall(struct trapframe *frame) {
    register int psig;
    time_t syst;
    int code;
    u_int sp;

    syst = u.u_ru.ru_stime;

    if ((unsigned) frame < (unsigned) &u + sizeof(u)) {
        panic("stack overflow");
        /* NOTREACHED */
    }

#ifdef UCB_METER
    cnt.v_trap++;
    cnt.v_syscall++;
#endif

    u.u_error = 0;
    u.u_frame = frame;
    u.u_code =
        u.u_frame->tf_pc - INSN_SZ; /* Syscall for sig handler. */

    led_control(LED_KERNEL, 1);

    /* Check stack. */
    sp = u.u_frame->tf_sp;
    if (sp < u.u_procp->p_daddr + u.u_dsize) {
        /* Process has trashed its stack; give it an illegal
	 * instruction violation to halt it in its tracks. */
#ifdef DEBUG_PRINT_SYSCALLS
	printf("\tsyscall[%u]: bad stack: sp = 0x%08x daddr = 0x0%08x dsize = 0x%08x, sending SEGV\n",
	      u.u_procp->p_pid, sp, u.u_procp->p_daddr, u.u_dsize);
#endif
        psig = SIGSEGV;
        goto bad;
    }
    if (u.u_procp->p_ssize < (size_t) __user_data_end - sp) {
        /* Expand stack. */
        u.u_procp->p_ssize = (size_t) __user_data_end - sp;
        u.u_procp->p_saddr = sp;
        u.u_ssize          = u.u_procp->p_ssize;
    }

    code = *(int *) u.u_code & 0377; /* Bottom 8 bits are index. */

    const struct sysent *callp = &sysent[0];

    if (code < nsysent)
        callp += code;

    if (callp->sy_narg) {
        /* In AAPCS, first four args are from trapframe regs r0-r3. */
        u.u_arg[0]   = u.u_frame->tf_r0; /* $a1 */
        u.u_arg[1]   = u.u_frame->tf_r1; /* $a2 */
        u.u_arg[2]   = u.u_frame->tf_r2; /* $a3 */
        u.u_arg[3]   = u.u_frame->tf_r3; /* $a4 */

        /* In AAPCS, stack must be double-word aligned. */
        int stkalign = 0;
        if (u.u_frame->tf_psr & SCB_CCR_STKALIGN_MASK) {
            stkalign = 4; /* Skip over padding byte. */
        }

        /* Remaining args are from the stack, after the trapframe. */
        if (callp->sy_narg > 4) {
            u_int addr = (u.u_frame->tf_sp + 32 + stkalign) & ~3;
            if (!baduaddr((caddr_t) addr))
                u.u_arg[4] = *(u_int *) addr;
        }
        if (callp->sy_narg > 5) {
            u_int addr = (u.u_frame->tf_sp + 36 + stkalign) & ~3;
            if (!baduaddr((caddr_t) addr))
                u.u_arg[5] = *(u_int *) addr;
        }
    }
    u.u_rval = 0;

#ifdef DEBUG_PRINT_SYSCALLS
    printf("\tsyscall[%u]: %s (", u.u_procp->p_pid,
            syscallnames [code >= nsysent ? 0 : code]);
    if (callp->sy_narg > 0)
	print_args(code, callp->sy_narg, u.u_arg[0], u.u_arg[1],
		   u.u_arg[2], u.u_arg[3], u.u_arg[4], u.u_arg[5]);
    printf(")"); /* was: ") at %08x\n", pc); */
#endif
    
    if (setjmp(&u.u_qsave) == 0) {
        (*callp->sy_call)(); /* Make syscall. */
    }

    switch (u.u_error) {
    case 0:
        u.u_frame->tf_psr &= ~PSR_C; /* Clear carry bit. */
        u.u_frame->tf_r0 = u.u_rval; /* $a1 - result. */
        break;
    case ERESTART:
        u.u_frame->tf_pc -= INSN_SZ; /* Return to svc syscall. */
        break;
    case EJUSTRETURN: /* Return from sig handler. */
        break;
    default:
        u.u_frame->tf_psr |= PSR_C;   /* Set carry bit. */
        u.u_frame->tf_r0 = u.u_error; /* $a1 - result. */
        break;
    }
#ifdef DEBUG_PRINT_SYSCALLS
    switch (u.u_error) {
    case ERESTART:
	printf(" -> restart");
        break;
    case EJUSTRETURN: /* Return from sig handler. */
	printf(" -> just return");
        break;
    default:
        printf(" = %u", u.u_frame->tf_r0);
        break;
    }
    printf("\n");
#endif

    goto out;

bad:
    /* From this point and further the interrupts must be enabled. */
    (void) arm_enable_interrupts();

    psignal(u.u_procp, psig);

out:
    (void) arm_enable_interrupts();

    userret(u.u_frame->tf_pc, syst);

    led_control(LED_KERNEL, 0);
}
