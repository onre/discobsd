/*
* This is port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3 
* v1.0
* Mark Pendrith, Nov 27, 2012.
*
* From Mark:
* >When I ported the macros I emailed Dean to ask what attribution would be
* >appropriate, and here is his response:
* >
* >>Mark,
* >>I think it's great that you've ported the macros; consider them
* >>public domain, to do with whatever you wish. I hope you find them >useful .
* >>
* >>Cheers!
* >>- Dean 
*/

#ifdef __arm__
#ifndef _CORTEX_M3_ATOMIC_H_
#define _CORTEX_M3_ATOMIC_H_

static __inline__ unsigned int __get_primask(void)
    __attribute__((always_inline));
static __inline__ unsigned int __get_primask(void) {
    unsigned int primask = 0;
    asm volatile("MRS %[result], PRIMASK\n\t"
                     : [result] "=r"(primask)::);
    return primask;
} // returns 0 if interrupts enabled, 1 if disabled

static __inline__ void __set_primask(unsigned int setval)
    __attribute__((always_inline));
static __inline__ void __set_primask(unsigned int setval) {
    asm volatile("MSR PRIMASK, %[value]\n\t"
                     "dmb\n\t"
                     "dsb\n\t"
                     "isb\n\t" ::[value] "r"(setval)
                     :);
    asm volatile("" ::: "memory");
}

static __inline__ void __enable_irq_set_barrier(void);
static __inline__ void __enable_irq_set_barrier(void) {
    asm volatile("CPSIE i");
    asm volatile("dmb");
    asm volatile("dsb");
    asm volatile("isb");
}

static __inline__ void __set_barrier(void);
static __inline__ void __set_barrier(void) {
    asm volatile("dmb");
    asm volatile("dsb");
    asm volatile("isb");
}

static __inline__ void __disable_irq_set_barrier(void);
static __inline__ void __disable_irq_set_barrier(void) {
    asm volatile("CPSID i");
    asm volatile("dmb");
    asm volatile("dsb");
    asm volatile("isb");
}



static __inline__ unsigned int __iSeiRetVal(void)
    __attribute__((always_inline));
static __inline__ unsigned int __iSeiRetVal(void) {
    asm volatile("CPSIE i\n\t"
                     "dmb\n\t"
                     "dsb\n\t"
                     "isb\n\t");
    asm volatile("" ::: "memory");
    return 1;
}

static __inline__ unsigned int __iCliRetVal(void)
    __attribute__((always_inline));
static __inline__ unsigned int __iCliRetVal(void) {
    asm volatile("CPSID i\n\t"
                     "dmb\n\t"
                     "dsb\n\t"
                     "isb\n\t");
    asm volatile("" ::: "memory");
    return 1;
}

static __inline__ void __iSeiParam(const unsigned int *__s)
    __attribute__((always_inline));
static __inline__ void __iSeiParam(const unsigned int *__s) {
    asm volatile("CPSIE i\n\t"
                     "dmb\n\t"
                     "dsb\n\t"
                     "isb\n\t");
    asm volatile("" ::: "memory");
    (void) __s;
}

static __inline__ void __iCliParam(const unsigned int *__s)
    __attribute__((always_inline));
static __inline__ void __iCliParam(const unsigned int *__s) {
    asm volatile("CPSID i\n\t"
                     "dmb\n\t"
                     "dsb\n\t"
                     "isb\n\t");
    asm volatile("" ::: "memory");
    (void) __s;
}

static __inline__ void __iRestore(const unsigned int *__s)
    __attribute__((always_inline));
static __inline__ void __iRestore(const unsigned int *__s) {
    __set_primask(*__s);
    asm volatile("" ::: "memory");
}


#define ATOMIC_BLOCK(type)                                             \
    for (type, __ToDo = __iCliRetVal(); __ToDo; __ToDo = 0)

#define ATOMIC_RESTORESTATE                                            \
    unsigned int primask_save                                          \
        __attribute__((__cleanup__(__iRestore))) = __get_primask()

#define ATOMIC_FORCEON                                                 \
    unsigned int primask_save                                          \
        __attribute__((__cleanup__(__iSeiParam))) = 0

#define NONATOMIC_BLOCK(type)                                          \
    for (type, __ToDo = __iSeiRetVal(); __ToDo; __ToDo = 0)

#define NONATOMIC_RESTORESTATE                                         \
    unsigned int primask_save                                          \
        __attribute__((__cleanup__(__iRestore))) = __get_primask()

#define NONATOMIC_FORCEOFF                                             \
    unsigned int primask_save                                          \
        __attribute__((__cleanup__(__iCliParam))) = 0

#endif
#endif
