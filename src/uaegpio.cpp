/*
* UAE - The Un*x Amiga Emulator
*
* uaegpio.device
*
* Copyright 2020 Johannes Lundberg
*
*/

/*
* uaegpio.device gives the emulated Amiga (indirect) access to the host's
* GPIO ports using /dev/gpiomem. The primary target for this library is
* the Raspberry Pi and might need adapting to work on other platforms.
*
* Based on fs-uae's uaeserial.device source code.
*
* The routines for accessing the host GPIO memory is inspired by pigpio's 
* public domain Tiny GPIO example. 
*
* http://abyz.me.uk/rpi/pigpio/examples.html#Misc_tiny_gpio
*/

/*
Call order at boot
------------------
uaegpio: uaegpiodev_install
uaegpio: uaepgiodev_reset
uaegpio: dev_reset
uaegpio: uaegpiodev_start_threads
uaegpio: uaegpiodev_startup
uaegpio: dev_init

Program accesses OpenDevice() etc
uaegpio: dev_open
uaegpio: dev_close

 */

#include "sysconfig.h"
#include "sysdeps.h"

#ifdef UAEGPIO

#include "threaddep/thread.h"
#include "options.h"
#include "memory.h"
#include "custom.h"
#include "newcpu.h"
#include "traps.h"
#include "autoconf.h"
#include "execlib.h"
#include "native2amiga.h"
#include "uaegpio.h"
#include "execio.h"


/* 
 * START Raspberry Pi GPIO code
 */
/* 
 * END Raspberry Pi GPIO code
 */

#define MAX_TOTAL_DEVICES 8

#ifdef AMIBERRY
int log_uaegpio = 1;
#else
int log_uaegpio = 0;
#endif


/* 
 * I/O commands for GPIO 
 */
#define GPIO_READ	11
#define GPIO_WAIT	12
#define GPIO_WRITE	13
#define GPIO_CONFIG	14

#ifdef AMIBERRY // NL
#define asyncreq uaegpio_asyncreq
#define devstruct uaegpio_devstruct
#endif

struct asyncreq {
	struct asyncreq *next;
	uaecptr arequest;
	uae_u8 *request;
	int ready;
};

struct devstruct {
	int open;
	int unit;
	int uniq;
	int exclusive;

	struct asyncreq *ar;

	smp_comm_pipe requests;
	int thread_running;
	uae_sem_t sync_sem;

	void *sysdata;
};

static int uniq;
static uae_u32 nscmd_cmd;
static struct devstruct devst[MAX_TOTAL_DEVICES];
static uae_sem_t change_sem, async_sem, pipe_sem;

static const TCHAR *getdevname (void)
{
	return _T("uaegpio.device");
}

static void io_log (const TCHAR *msg, uae_u8 *request, uaecptr arequest)
{
	if (log_uaegpio)
		write_log (_T("%s: %p %d %08X %d %d io_actual=%d io_error=%d\n"),
		msg, request, get_word_host(request + 28), get_long_host(request + 40),
			get_long_host(request + 36), get_long_host(request + 44),
			get_long_host(request + 32), get_byte_host(request + 31));
}

static struct devstruct *getdevstruct (int uniq)
{
	int i;
	for (i = 0; i < MAX_TOTAL_DEVICES; i++) {
		if (devst[i].uniq == uniq)
			return &devst[i];
	}
	return 0;
}

static int dev_thread (void *devs);
static int start_thread (struct devstruct *dev)
{
	printf("uaegpio: start_thread. open %d, unit %d\n", dev->open, dev->unit);

	init_comm_pipe (&dev->requests, 100, 1);
	uae_sem_init (&dev->sync_sem, 0, 0);
	uae_start_thread (_T("uaegpio"), dev_thread, dev, NULL);
	uae_sem_wait (&dev->sync_sem);
	return dev->thread_running;
}

static void dev_close_3 (struct devstruct *dev)
{
	// uaeser_close (dev->sysdata);
	dev->open = 0;
	xfree (dev->sysdata);
	uae_sem_wait(&pipe_sem);
	write_comm_pipe_pvoid(&dev->requests, NULL, 0);
	write_comm_pipe_pvoid(&dev->requests, NULL, 0);
	write_comm_pipe_u32 (&dev->requests, 0, 1);
	uae_sem_post(&pipe_sem);
}

static uae_u32 REGPARAM2 dev_close (TrapContext *ctx)
{
    printf("uaegpio: dev_close\n");
    
	uae_u32 request = trap_get_areg(ctx, 1);
	struct devstruct *dev;

	dev = getdevstruct (trap_get_long(ctx, request + 24));
	if (!dev)
		return 0;
	if (log_uaegpio)
		write_log (_T("%s:%d close, req=%x\n"), getdevname(), dev->unit, request);
	dev_close_3 (dev);
	trap_put_long(ctx, request + 24, 0);
	trap_put_word(ctx, trap_get_areg(ctx, 6) + 32, trap_get_word(ctx, trap_get_areg(ctx, 6) + 32) - 1);
	return 0;
}


static int openfail(TrapContext *ctx, uaecptr ioreq, int error)
{
	trap_put_long(ctx, ioreq + 20, -1);
	trap_put_byte(ctx, ioreq + 31, error);
	return (uae_u32)-1;
}

static uae_u32 REGPARAM2 dev_open (TrapContext *ctx)
{
	printf("uaegpio: dev_open\n");

	uaecptr ioreq = trap_get_areg(ctx, 1);
	uae_u32 unit = trap_get_dreg(ctx, 0);
	uae_u32 flags = trap_get_dreg(ctx, 1);
	struct devstruct *dev;
	int i, err = 0;
	uae_u8 request[IOSTDREQ_SIZE];

	trap_get_bytes(ctx, request, ioreq, IOSTDREQ_SIZE);
    
	if (trap_get_word(ctx, ioreq + 0x12) < IOSTDREQ_SIZE)
		return openfail(ctx, ioreq, IOERR_BADLENGTH);
	for (i = 0; i < MAX_TOTAL_DEVICES; i++) {
		if (devst[i].open && devst[i].unit == unit && devst[i].exclusive)
			return openfail(ctx, ioreq, IOERR_UNITBUSY);
	}
	for (i = 0; i < MAX_TOTAL_DEVICES; i++) {
		if (!devst[i].open)
			break;
	}
	if (i == MAX_TOTAL_DEVICES)
		return openfail(ctx, ioreq, IOERR_OPENFAIL);
	dev = &devst[i];
	// dev->sysdata = xcalloc (uae_u8, uaeser_getdatalength ());
	// if (!uaeser_open (dev->sysdata, dev, unit)) {
	// 	xfree (dev->sysdata);
	// 	return openfail(ctx, ioreq, IOERR_OPENFAIL);
	// }
	dev->unit = unit;
	dev->open = 1;
	dev->uniq = ++uniq;
	dev->exclusive = 0;
	put_long_host(request + 24, dev->uniq);
	// resetparams (ctx, dev, request);
	// err = setparams (ctx, dev, request);
	// if (err) {
		// uaeser_close (dev->sysdata);
		// dev->open = 0;
		// xfree (dev->sysdata);
		// return openfail(ctx, ioreq, err);
	// }
	if (log_uaegpio)
		write_log (_T("%s:%d open ioreq=%08X\n"), getdevname(), unit, ioreq);
	start_thread (dev);

	trap_put_word(ctx, trap_get_areg(ctx, 6) + 32, trap_get_word(ctx, trap_get_areg(ctx, 6) + 32) + 1);
	put_byte_host(request + 31, 0);
	put_byte_host(request + 8, 7);
	trap_put_bytes(ctx, request + 8, ioreq + 8, IOSTDREQ_SIZE - 8);

	return 0;
}

static uae_u32 REGPARAM2 dev_expunge (TrapContext *context)
{
    printf("uaegpio: dev_expunge\n");
    return 0;
}

static struct asyncreq *get_async_request (struct devstruct *dev, uaecptr arequest, int ready)
{
	printf("%s uae_sem_wait(async_sem)\n", __func__);

	struct asyncreq *ar;

	uae_sem_wait (&async_sem);
	printf("%s uae_sem_wait(async_sem) done\n", __func__);
	ar = dev->ar;
	while (ar) {
		if (ar->arequest == arequest) {
			if (ready)
				ar->ready = 1;
			break;
		}
		ar = ar->next;
	}
	printf("%s uae_sem_post(async_sem)\n", __func__);
	uae_sem_post (&async_sem);
	printf("%s uae_sem_post(async_sem) done\n", __func__);
	return ar;
}

static int add_async_request (struct devstruct *dev, uae_u8 *request, uaecptr arequest)
{
	printf("%s\n", __func__);

	struct asyncreq *ar, *ar2;

	if (log_uaegpio)
		write_log (_T("%s:%d async request %x added\n"), getdevname(), dev->unit, arequest);

	uae_sem_wait (&async_sem);
	ar = xcalloc (struct asyncreq, 1);
	ar->arequest = arequest;
	ar->request = request;
	if (!dev->ar) {
		dev->ar = ar;
	} else {
		ar2 = dev->ar;
		while (ar2->next)
			ar2 = ar2->next;
		ar2->next = ar;
	}
	uae_sem_post (&async_sem);
	return 1;
}

static int release_async_request (struct devstruct *dev, uaecptr arequest)
{
	struct asyncreq *ar, *prevar;

	uae_sem_wait (&async_sem);
	ar = dev->ar;
	prevar = NULL;
	while (ar) {
		if (ar->arequest == arequest) {
			if (prevar == NULL)
				dev->ar = ar->next;
			else
				prevar->next = ar->next;
			uae_sem_post (&async_sem);
			xfree(ar->request);
			xfree(ar);
			if (log_uaegpio)
				write_log (_T("%s:%d async request %x removed\n"), getdevname(), dev->unit, arequest);
			return 1;
		}
		prevar = ar;
		ar = ar->next;
	}
	uae_sem_post (&async_sem);
	write_log (_T("%s:%d async request %x not found for removal!\n"), getdevname(), dev->unit, arequest);
	return 0;
}

static void abort_async(TrapContext *ctx, struct devstruct *dev, uaecptr arequest)
{
	printf("%s\n", __func__);

	struct asyncreq *ar = get_async_request (dev, arequest, 1);
	if (!ar) {
		write_log (_T("%s:%d: abort async but no request %x found!\n"), getdevname(), dev->unit, arequest);
		return;
	}
	uae_u8 *request = ar->request;
	if (log_uaegpio)
		write_log (_T("%s:%d asyncronous request=%08X aborted\n"), getdevname(), dev->unit, arequest);
	put_byte_host(request + 31, IOERR_ABORTED);
	put_byte_host(request + 30, get_byte_host(request + 30) | 0x20);
	uae_sem_wait(&pipe_sem);
	write_comm_pipe_pvoid(&dev->requests, ctx, 0);
	write_comm_pipe_pvoid(&dev->requests, request, 0);
	write_comm_pipe_u32(&dev->requests, arequest, 1);
	uae_sem_post(&pipe_sem);
}



static int dev_do_io(TrapContext *ctx, struct devstruct *dev, uae_u8 *request, uaecptr arequest, int quick)
{
    printf("uaegpio: dev_do_io\n");

	uae_u32 command;
	uae_u32 io_data = get_long_host(request + 40); // 0x28
	uae_u32 io_length = get_long_host(request + 36); // 0x24
	uae_u32 io_actual = get_long_host(request + 32); // 0x20
	uae_u32 io_offset = get_long_host(request + 44); // 0x2c
	uae_u32 io_error = 0;
	uae_u16 io_status;
	int async = 0;

    printf("uaegpio: dev_do_io: length %d\n", io_length);

	if (!dev)
		return 0;
	command = get_word_host(request + 28);
	io_log (_T("dev_io_START"), request, arequest);

	switch (command)
	{
	case GPIO_READ:
		printf("uaegpio: dev_do_io: GPIO_READ\n");
		break;
	case GPIO_WAIT:
		printf("uaegpio: dev_do_io: GPIO_WAIT\n");
		async = 1;
		break;
	case GPIO_WRITE:
		printf("uaegpio: dev_do_io: GPIO_WRITE\n");
		break;
	case GPIO_CONFIG:
		printf("uaegpio: dev_do_io: GPIO_CONFIG\n");
		break;
	case NSCMD_DEVICEQUERY:
		trap_put_long(ctx, io_data + 0, 0);
		trap_put_long(ctx, io_data + 4, 16); /* size */
		trap_put_word(ctx, io_data + 8, NSDEVTYPE_SERIAL);
		trap_put_word(ctx, io_data + 10, 0);
		trap_put_long(ctx, io_data + 12, nscmd_cmd);
		io_actual = 16;
		break;
	default:
		io_error = IOERR_NOCMD;
		break;
	}
	put_long_host(request + 32, io_actual);
	put_byte_host(request + 31, io_error);
	io_log (_T("dev_io_END"), request, arequest);
	return async;
}

static int dev_canquick (struct devstruct *dev, uae_u8 *request)
{
	uae_u32 command;
	int quick = 0;
	
	command = get_word_host(request + 28);
	switch (command)
	{
	case GPIO_READ:
		quick = 1;
		break;
	case GPIO_WAIT:
		quick = 0;
		break;
	case GPIO_WRITE:
		quick = 1;
		break;
	case GPIO_CONFIG:
		quick = 1;
		break;
	}
	return quick;
}

static uae_u32 REGPARAM2 dev_beginio (TrapContext *ctx)
{
	printf("uaegpio: dev_beginio\n");
    
	uae_u8 err = 0;
	uae_u32 arequest = trap_get_areg(ctx, 1);
	uae_u8 *request = xmalloc(uae_u8, IOSTDREQ_SIZE);

	trap_get_bytes(ctx, request, arequest, IOSTDREQ_SIZE);

	uae_u8 flags = get_byte_host(request + 30);
	int command = get_word_host(request + 28);
	struct devstruct *dev = getdevstruct (get_long_host(request + 24));

	put_byte_host(request + 8, NT_MESSAGE);
	if (!dev) {
		err = 32;
		goto end;
	}
	put_byte_host(request + 31, 0);
	if ((flags & 1) && dev_canquick(dev, request)) {
		if (dev_do_io(ctx, dev, request, arequest, 1))
			write_log (_T("device %s:%d command %d bug with IO_QUICK\n"), getdevname(), dev->unit, command);
		err = get_byte_host(request + 31);
	} else {
		put_byte_host(request + 30, get_byte_host(request + 30) & ~1);
		trap_put_bytes(ctx, request + 8, arequest + 8, IOSTDREQ_SIZE - 8);
		uae_sem_wait(&pipe_sem);
		trap_set_background(ctx);
		write_comm_pipe_pvoid(&dev->requests, ctx, 0);
		write_comm_pipe_pvoid(&dev->requests, request, 0);
		write_comm_pipe_u32(&dev->requests, arequest, 1);
		uae_sem_post(&pipe_sem);
		return 0;
	}
end:
	put_byte_host(request + 31, 32);
	trap_put_bytes(ctx, request + 8, arequest + 8, IOSTDREQ_SIZE - 8);
	xfree(request);
	return err;
}

static int dev_thread (void *devs)
{
    printf("uaegpio: dev_thread\n");

	struct devstruct *dev = (struct devstruct*)devs;

	uae_set_thread_priority (NULL, 1);
	dev->thread_running = 1;
	uae_sem_post (&dev->sync_sem);
	for (;;) {
		TrapContext *ctx = (TrapContext*)read_comm_pipe_pvoid_blocking(&dev->requests);
		uae_u8 *iobuf = (uae_u8*)read_comm_pipe_pvoid_blocking(&dev->requests);
		uaecptr request = (uaecptr)read_comm_pipe_u32_blocking (&dev->requests);
		uae_sem_wait (&change_sem);
		printf("dev_thread: Got request\n");
		if (!request) {
			printf("dev_thread: Got request - not valid\n");
			dev->thread_running = 0;
			uae_sem_post (&dev->sync_sem);
			uae_sem_post (&change_sem);
			return 0;
		} else if (get_async_request (dev, request, 1)) {
			printf("dev_thread: get async request done (request was ready), do repy msg\n");
			uae_ReplyMsg (request);
			release_async_request (dev, request);
		} else if (dev_do_io(ctx, dev, iobuf, request, 0) == 0) {
			printf("dev_thread: dev_do_io done, do reply msg\n");
			uae_ReplyMsg (request);
		} else {
			printf("uaegpio: dev_thread: add async request (dev_do_io return 1=async)\n");
			add_async_request (dev, iobuf, request);
			// uaeser_trigger (dev->sysdata);
		}
		trap_background_set_complete(ctx);
		uae_sem_post (&change_sem);
	}
	return 0;
}

static uae_u32 REGPARAM2 dev_init (TrapContext *context)
{
    printf("uaegpio: dev_init\n");
	// if (gpioInitialise() < 0)
		// return 0;
	uae_u32 base = trap_get_dreg (context, 0);
	if (log_uaegpio)
		write_log (_T("%s init\n"), getdevname ());
	return base;
}

static uae_u32 REGPARAM2 dev_abortio(TrapContext *ctx)
{
    printf("uaegpio: dev_abortio\n");
    return 0;
	uae_u32 request = trap_get_areg(ctx, 1);
	struct devstruct *dev = getdevstruct(trap_get_long(ctx, request + 24));

	if (!dev) {
		trap_put_byte(ctx, request + 31, 32);
		return trap_get_byte(ctx, request + 31);
	}
	abort_async(ctx, dev, request);
	return 0;
}

static void dev_reset (void)
{
    printf("uaegpio: dev_reset\n");

	int i;
	struct devstruct *dev;

	for (i = 0; i < MAX_TOTAL_DEVICES; i++) {
		dev = &devst[i];
		if (dev->open) {
			while (dev->ar)
				abort_async(NULL, dev, dev->ar->arequest);
			dev_close_3 (dev);
			uae_sem_wait (&dev->sync_sem);
		}
		memset (dev, 0, sizeof (struct devstruct));
	}
}

static uaecptr ROM_uaegpiodev_resname = 0,
	ROM_uaegpiodev_resid = 0,
	ROM_uaegpiodev_init = 0;

uaecptr uaegpiodev_startup(TrapContext *ctx, uaecptr resaddr)
{
	printf("uaegpio: uaegpiodev_startup\n");

	if (!currprefs.uaegpio)
		return resaddr;
	if (log_uaegpio)
		write_log (_T("uaegpiodev_startup(0x%x)\n"), resaddr);
	/* Build a struct Resident. This will set up and initialize
	* the uaegpio.device */
	trap_put_word(ctx, resaddr + 0x0, 0x4AFC);
	trap_put_long(ctx, resaddr + 0x2, resaddr);
	trap_put_long(ctx, resaddr + 0x6, resaddr + 0x1A); /* Continue scan here */
	if (kickstart_version >= 37) {
		trap_put_long(ctx, resaddr + 0xA, 0x84010300 | AFTERDOS_PRI); /* RTF_AUTOINIT, RT_VERSION NT_LIBRARY, RT_PRI */
	} else {
		trap_put_long(ctx, resaddr + 0xA, 0x81010305); /* RTF_AUTOINIT, RT_VERSION NT_LIBRARY, RT_PRI */
	}
	trap_put_long(ctx, resaddr + 0xE, ROM_uaegpiodev_resname);
	trap_put_long(ctx, resaddr + 0x12, ROM_uaegpiodev_resid);
	trap_put_long(ctx, resaddr + 0x16, ROM_uaegpiodev_init);
	resaddr += 0x1A;
	return resaddr;
}


void uaegpiodev_install (void)
{
    printf("uaegpio: uaegpiodev_install\n");

	uae_u32 functable, datatable;
	uae_u32 initcode, openfunc, closefunc, expungefunc;
	uae_u32 beginiofunc, abortiofunc;

	if (!currprefs.uaegpio)
		return;

	ROM_uaegpiodev_resname = ds (_T("uaegpio.device"));
	ROM_uaegpiodev_resid = ds (_T("UAE gpio.device 0.1"));

	/* initcode */
	initcode = here ();
	calltrap (deftrap (dev_init)); dw (RTS);

	/* Open */
	openfunc = here ();
	calltrap (deftrap (dev_open)); dw (RTS);

	/* Close */
	closefunc = here ();
	calltrap (deftrap (dev_close)); dw (RTS);

	/* Expunge */
	expungefunc = here ();
	calltrap (deftrap (dev_expunge)); dw (RTS);

	/* BeginIO */
	beginiofunc = here ();
	calltrap (deftrap (dev_beginio)); dw (RTS);

	/* AbortIO */
	abortiofunc = here ();
	calltrap (deftrap (dev_abortio)); dw (RTS);

	/* FuncTable */
	functable = here ();
	dl (openfunc); /* Open */
	dl (closefunc); /* Close */
	dl (expungefunc); /* Expunge */
	dl (EXPANSION_nullfunc); /* Null */
	dl (beginiofunc); /* BeginIO */
	dl (abortiofunc); /* AbortIO */
	dl (0xFFFFFFFFul); /* end of table */

	/* DataTable */
	datatable = here ();
	dw (0xE000); /* INITBYTE */
	dw (0x0008); /* LN_TYPE */
	dw (0x0300); /* NT_DEVICE */
	dw (0xC000); /* INITLONG */
	dw (0x000A); /* LN_NAME */
	dl (ROM_uaegpiodev_resname);
	dw (0xE000); /* INITBYTE */
	dw (0x000E); /* LIB_FLAGS */
	dw (0x0600); /* LIBF_SUMUSED | LIBF_CHANGED */
	dw (0xD000); /* INITWORD */
	dw (0x0014); /* LIB_VERSION */
	dw (0x0004); /* 0.4 */
	dw (0xD000); /* INITWORD */
	dw (0x0016); /* LIB_REVISION */
	dw (0x0000);
	dw (0xC000); /* INITLONG */
	dw (0x0018); /* LIB_IDSTRING */
	dl (ROM_uaegpiodev_resid);
	dw (0x0000); /* end of table */

	ROM_uaegpiodev_init = here ();
	dl (0x00000100); /* size of device base */
	dl (functable);
	dl (datatable);
	dl (initcode);

	// Is this a list of supported functions??
	nscmd_cmd = here ();
	dw (NSCMD_DEVICEQUERY);
	dw (GPIO_READ);	
	dw (GPIO_WAIT);
	dw (GPIO_WRITE);
	dw (GPIO_CONFIG);
	dw (0);
}

void uaegpiodev_start_threads (void)
{
    printf("uaegpio: uaegpiodev_start_threads\n");

	uae_sem_init(&change_sem, 0, 1);
	uae_sem_init(&async_sem, 0, 1);
	uae_sem_init(&pipe_sem, 0, 1);
}

void uaegpiodev_reset (void)
{
    printf("uaegpio: uaepgiodev_reset\n");

	if (!currprefs.uaegpio)
		return;
	dev_reset ();
}

#endif /* UAEGPIO */
