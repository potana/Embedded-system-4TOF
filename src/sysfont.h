#ifndef SYSFONT_H
#define SYSFONT_H

#include <stddef.h>
#include <stdint.h>

/* The sysfont dictionary: {"Width", "Height", "Start", "End", "Data"} */
typedef struct {
    int width, height, start, end;
    const uint8_t *data;
    size_t data_len;
} sysfont_t;

extern const sysfont_t sysfont;

#endif
