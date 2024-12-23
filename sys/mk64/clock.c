/*
 * Copyright (c) 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */
#include <sys/param.h>
#include <machine/systick.h>

/*
 * Setup core timer for `hz' timer interrupts per second.
 */
void
clkstart()
{
    printf("systick: enabling\n");
    systick_init();
}
