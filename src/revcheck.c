#include "revcheck.h"

#include <stdint.h>

int pd_rev = PD_REV_UNDEFINED;

int rev_bss[] = {
    0x60,  // rev A
    // TODO
};

int rev_stack[] = {
    0x20,  // rev A
    // TODO
};

__attribute__((constructor)) void pd_revcheck(void)
{
#ifdef TARGET_SIMULATOR
    pd_rev = PD_REV_SIMULATOR;
#else
    volatile uintptr_t bss = (uintptr_t)(void *)&pd_rev;
    uintptr_t stack = (uintptr_t)(void *)&bss;

    int i = 0;
    for (i = 0; i < sizeof(rev_bss) / sizeof(rev_bss[0]); ++i)
    {
        if (bss >> 24 == rev_bss[i] && stack >> 24 == rev_stack[i])
        {
            pd_rev = i + 1;
            return;
        }
    }
    pd_rev = PD_REV_UNKNOWN;
#endif
}