#ifndef _MACHINE_SYSTICK_H_
#define _MACHINE_SYSTICK_H_

#include <sys/types.h>

extern volatile u_long systick_ms;
void systick_init(void);

#endif
