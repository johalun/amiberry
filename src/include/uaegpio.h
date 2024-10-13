 /*
  * UAE - The Un*x Amiga Emulator
  *
  * uaegpio.device
  *
  * (c) 2020 Johannes Lundberg
  */

#ifndef UAE_UAEGPIO_H
#define UAE_UAEGPIO_H

#include "uae/types.h"

uaecptr uaegpiodev_startup(TrapContext*, uaecptr resaddr);
void uaegpiodev_install(void);
void uaegpiodev_reset(void);
void uaegpiodev_start_threads(void);

extern int log_uaegpio;

struct uaegpiodata
{
#ifdef _WIN32
    void *handle;
    void *writeevent;
#endif
};

#endif /* UAE_UAEGPIO_H */
