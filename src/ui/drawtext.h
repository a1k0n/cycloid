#ifndef UI_DRAWTEXT_H_
#define UI_DRAWTEXT_H_

#include <stdint.h>

extern void DrawText(const char *str, int x, int y, uint16_t color,
    uint16_t *buf);

#endif  // UI_DRAWTEXT_H_
