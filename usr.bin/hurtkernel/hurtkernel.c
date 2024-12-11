#include <sys/types.h>

int main()
{
    for (u_int *p = (u_int *) 0x1fff0000; p < (u_int *) 0x20000000; p++)
	*p = 0;

    return 0; /* leap of faith */
}
