#ifndef HW_GPIO_GPIO_H_
#define HW_GPIO_GPIO_H_

// I/O access
extern volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or
// SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7 << (((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1 << (((g)%10)*3))
#define SET_GPIO_ALT(g, a) *(gpio+(((g)/10))) |= \
            (((a) <= 3?(a)+4:(a) == 4?3:2) << (((g)%10)*3))

#define GPIO_SET *(gpio+7)   // sets bits which are 1
#define GPIO_CLR *(gpio+10)  // clears bits which are 1

#define GPIO_IN0   *(gpio+13)  // Reads GPIO input bits 0-31

#define GPIO_PULL   *(gpio+37)    // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38)  // Pull up/pull down clock

#define GET_GPIO(g) (*(gpio+13) & (1 << g))  // 0 if LOW, (1<<g) if HIGH

bool gpio_init();  // N.B. must be root

#endif  // HW_GPIO_GPIO_H_
