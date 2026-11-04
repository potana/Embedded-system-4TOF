/* Python-style exceptions for the C port, using setjmp/longjmp.
 *
 * 4ToF.py relies on exceptions: an I2C failure raises OSError, which
 * safe_read() turns into None, while a RuntimeError / TimeoutError (or an
 * OSError outside safe_read) ends the script.  The C code keeps that control
 * flow:
 *
 *     exc_frame_t f;
 *     exc_push(&f);
 *     if (setjmp(f.buf) == 0) {
 *         ... code that may raise ...
 *         exc_pop(&f);
 *     } else {
 *         ... handler: f.type / f.err / f.msg say what was raised ...
 *     }
 *
 * A raise with no frame left prints the error like MicroPython's traceback
 * and stops the program.
 */
#ifndef PYEXC_H
#define PYEXC_H

#include <setjmp.h>

typedef enum {
    EXC_OSERROR = 1,     /* I2C errors (errno in .err) */
    EXC_RUNTIME,         /* RuntimeError, and the VL53L0X module's TimeoutError */
} exc_type_t;

typedef struct exc_frame {
    jmp_buf buf;
    struct exc_frame *prev;
    exc_type_t type;
    int err;
    const char *msg;
} exc_frame_t;

#define PY_ENODEV    19
#define PY_ETIMEDOUT 110

void exc_push(exc_frame_t *f);
void exc_pop(exc_frame_t *f);

void raise_oserror(int err) __attribute__((noreturn));
void raise_runtime(const char *msg) __attribute__((noreturn));
/* Pass a caught exception on to the next handler. */
void exc_reraise(const exc_frame_t *f) __attribute__((noreturn));

#endif
