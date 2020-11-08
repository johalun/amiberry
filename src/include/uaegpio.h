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

/* 
 * I/O commands for GPIO 
 */
#define GPIO_READ	11
#define GPIO_WAIT	12
#define GPIO_WRITE	13
#define GPIO_CONFIG	14


enum GPIOState {
    GPIO_High,
    GPIO_Low,
};
enum GPIODirection {
    GPIO_Input,
    GPIO_Output
};
enum GPIOTrigger {
    GPIO_Rising,
    GPIO_Falling
};
enum GPIOPullUpDown {
    GPIO_None,
    GPIO_PullUp,
    GPIO_PullDown,
};

struct GPIORead {
    unsigned char channel;		/* GPIO pin number (BCM) to read. */
    unsigned char value;		/* Result returned here */
};

struct GPIOWait {
    unsigned char channel;				/* GPIO pin number (BCM) to async wait for trigger. */
    enum GPIOTrigger trigger;	/* Rising or Falling */
};

struct GPIOWrite {
    unsigned char channel;			/* GPIO pin number (BCM) to write. */
    unsigned char value;			/* High or Low */
};

struct GPIOConfig {
    unsigned char channel;					/* GPIO pin number (BCM) to configure. */
    enum GPIODirection direction;	/* Input or Output. */
    enum GPIOPullUpDown pud;		/* None, Up or Down */
};

#endif /* UAE_UAEGPIO_H */
