#ifndef UI_DRAWTEXT_H_
#define UI_DRAWTEXT_H_

#include <stdint.h>

// draw 8x10 font
extern void DrawText(const char *str, int x, int y, uint16_t color,
    uint16_t *buf);

// draw 16x20 font
extern void DrawTextBig(const char *str, int x, int y, uint16_t color,
    uint16_t *buf);

extern int TextWidthBig(const char *str);

#endif  // UI_DRAWTEXT_H_
