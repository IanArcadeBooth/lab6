// lab06_step3_3led.c
// Step 3: GPIO memory-mapped control (3 LEDs + 3 buttons)
// Based on Appendix A mmap(/dev/mem) example.

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#define LED_ON 1
#define LED_OFF 0

#define GPIO_BASE 0xFE200000 //(for Pi 4)

#define GPFSEL0 0
#define GPSET0 7
#define GPCLR0 10
#define GPLEV0 13
#define GPPUD 37
#define GPPUDCLK0 38

volatile unsigned int *GPIO;

// 🔧 CHANGE THESE TO MATCH YOUR WIRING
int LED0 = 17;
int LED1 = 18;
int LED2 = 27;

int B0 = 22; // right -> left
int B1 = 23; // left -> right
int B2 = 24; // stop

void gpio_mode(int pin, int output) {
  int reg = pin / 10;
  int shift = (pin % 10) * 3;
  GPIO[GPFSEL0 + reg] &= ~(7 << shift);
  if (output)
    GPIO[GPFSEL0 + reg] |= (1 << shift);
}

void gpio_write(int pin, int val) {
  if (val)
    GPIO[GPSET0] = (1 << pin);
  else
    GPIO[GPCLR0] = (1 << pin);
}

int gpio_read(int pin) { return (GPIO[GPLEV0] & (1 << pin)) != 0; }

void pulldown(int pin) {
  GPIO[GPPUD] = 1;
  usleep(1);
  GPIO[GPPUDCLK0] = (1 << pin);
  usleep(1);
  GPIO[GPPUD] = 0;
  GPIO[GPPUDCLK0] = 0;
}

int main() {
  int fd;

  if (getuid() != 0) {
    printf("Run with sudo\n");
    return 1;
  }

  fd = open("/dev/mem", O_RDWR | O_SYNC);
  GPIO = (unsigned int *)mmap(0, getpagesize(), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, GPIO_BASE);

  // LEDs = outputs
  gpio_mode(LED0, 1);
  gpio_mode(LED1, 1);
  gpio_mode(LED2, 1);

  // Buttons = inputs + pulldown
  gpio_mode(B0, 0);
  pulldown(B0);
  gpio_mode(B1, 0);
  pulldown(B1);
  gpio_mode(B2, 0);
  pulldown(B2);

  int dir = 1; // 1 = left->right, 0 = right->left
  int pos = 0;

  while (1) {
    // read buttons (don’t require holding)
    if (gpio_read(B0))
      dir = 0;
    if (gpio_read(B1))
      dir = 1;
    if (gpio_read(B2))
      break;

    // LEDs off
    gpio_write(LED0, LED_OFF);
    gpio_write(LED1, LED_OFF);
    gpio_write(LED2, LED_OFF);

    // one LED on
    if (pos == 0)
      gpio_write(LED0, LED_ON);
    if (pos == 1)
      gpio_write(LED1, LED_ON);
    if (pos == 2)
      gpio_write(LED2, LED_ON);

    // advance every loop (marquee!)
    if (dir)
      pos = (pos + 1) % 3;
    else
      pos = (pos + 2) % 3; // equivalent to -1 mod 3

    usleep(250000);
  }

  return 0;
}
