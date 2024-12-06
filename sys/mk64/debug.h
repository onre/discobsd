#ifndef _DEBUG_H
#define _DEBUG_H

#define LEDPATTERN1                                                    \
    int xyz = 1, xyzd = 1;                                             \
    while (1) {                                                        \
        volatile int i;                                                \
        for (i = 0; i < 0x18ffff; i++)                                 \
            ;                                                          \
        teensy_gpio_led_value(xyz);				       \
        if (xyzd)                                                      \
            xyz = xyz << 1;                                            \
        else                                                           \
            xyz = xyz >> 1;                                            \
        if (xyz == 1)                                                  \
            xyzd = 1;                                                  \
        else if (xyz & (1 << 7))                                       \
            xyzd = 0;                                                  \
    }


#if 0
#define DEBUG_PRINT_SYSCALLS
#endif

#define DEBUG(...)
#define DEBUG1(...)
#define DEBUG2(...)
#define DEBUG3(...)
#define DEBUG4(...)
#define DEBUG5(...)
#define DEBUG6(...)
#define DEBUG7(...)
#define DEBUG8(...)
#define DEBUG9(...)

#ifdef GLOBAL_DEBUG
#undef DEBUG
#define DEBUG(...) printf(__VA_ARGS__)

#if (GLOBAL_DEBUG >= 1)
#undef DEBUG1
#define DEBUG1(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 2)
#undef DEBUG2
#define DEBUG2(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 3)
#undef DEBUG3
#define DEBUG3(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 4)
#undef DEBUG4
#define DEBUG4(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 5)
#undef DEBUG5
#define DEBUG5(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 6)
#undef DEBUG6
#define DEBUG6(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 7)
#undef DEBUG7
#define DEBUG7(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 8)
#undef DEBUG8
#define DEBUG8(...) printf(__VA_ARGS__)
#endif

#if (GLOBAL_DEBUG >= 9)
#undef DEBUG9
#define DEBUG9(...) printf(__VA_ARGS__)
#endif

#endif /* GLOBAL_DEBUG */

#endif /* _DEBUG_H */
