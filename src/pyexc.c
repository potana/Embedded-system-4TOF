#include <stdio.h>
#include "pico/stdlib.h"
#include "pyexc.h"

static exc_frame_t *top;

void exc_push(exc_frame_t *f)
{
    f->prev = top;
    top = f;
}

void exc_pop(exc_frame_t *f)
{
    top = f->prev;
}

/* Uncaught: the script has ended.  MicroPython prints the traceback once and
 * goes back to the REPL; here the message is repeated every 2 s so it is
 * still seen if the serial monitor is opened later. */
static void uncaught(exc_type_t type, int err, const char *msg) __attribute__((noreturn));
static void uncaught(exc_type_t type, int err, const char *msg)
{
    for (;;) {
        if (type == EXC_OSERROR) printf("OSError: [Errno %d]\n", err);
        else                     printf("RuntimeError: %s\n", msg);
        sleep_ms(2000);
    }
}

static void raise(exc_type_t type, int err, const char *msg) __attribute__((noreturn));
static void raise(exc_type_t type, int err, const char *msg)
{
    exc_frame_t *f = top;
    if (!f) uncaught(type, err, msg);
    top = f->prev;
    f->type = type;
    f->err = err;
    f->msg = msg;
    longjmp(f->buf, 1);
}

void raise_oserror(int err)          { raise(EXC_OSERROR, err, NULL); }
void raise_runtime(const char *msg)  { raise(EXC_RUNTIME, 0, msg); }
void exc_reraise(const exc_frame_t *f) { raise(f->type, f->err, f->msg); }
