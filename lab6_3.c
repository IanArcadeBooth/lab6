/*
 * File: lab6_3.c
 * Author: Ian Booth
 * Date: 2026/02/25
 * Description: This file controls a 3 led marquee effect
 * using memory-mapped GPIO access (/dev/mem). three buttons control direction
 * ending the program
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#define LED_ON 1
#define LED_OFF 0
#define SLEEP_TIME 250000

#define GPIO_BASE 0xFE200000 //(for Pi 4)

#define GPFSEL0 0
#define GPSET0 7
#define GPCLR0 10
#define GPLEV0 13
#define GPPUD 37
#define GPPUDCLK0 38

volatile unsigned int *GPIO;

int LED0 = 17; // physical pin 11
int LED1 = 18; // physical pin 12
int LED2 = 27; // physical pin 13

int B0 = 22; //(physical pin 15) right
int B1 = 23; //(physical pin 16) left
int B2 = 24; //(phsycial pin 18) stop

/*
 * Description: Configures a GPIO pin as an input or output.
 * @param pin: GPIO number to configure.
 * @param output: 1 to set as output, 0 to set as input.
 * @return: none
 * side effects: Modifies the GPIO function select register for the pin.
 */
void gpio_mode(int pin, int output) {
  int reg = pin / 10;                   // gpfsel controls 10 gpio pins
  int shift = (pin % 10) * 3;           // 3 bits for each pin
  GPIO[GPFSEL0 + reg] &= ~(7 << shift); // clears bits
  if (output)
    GPIO[GPFSEL0 + reg] |= (1 << shift); // sets the output to 001
}

/*
 * Description: Writes a high or low to GPIO pins
 * @param pin: GPIO number to write.
 * @param val: Output value (0 = LOW, non-zero = HIGH).
 * @return: none
 * side effects: Drives the specified GPIO pin HIGH or LOW.
 */
void gpio_write(int pin, int val) {
  if (val)
    GPIO[GPSET0] = (1 << pin); // if value is non zero turns the pin on
  else
    GPIO[GPCLR0] = (1 << pin); // if value is 0 sets pin low
}

/*
 * Description: Reads the digital value from a GPIO input pin.
 * @param pin: GPIO number to read.
 * @return: 1 if pin is HIGH, 0 if pin is LOW.
 * side effects: Reads GPIO register.
 */
int gpio_read(int pin) { return (GPIO[GPLEV0] & (1 << pin)) != 0; }

/*
 * Description: Enables the pull-down resistor on a GPIO pin.
 * @param pin: BCM GPIO number to configure.
 * @return: none
 * side effects: Changes configuration of the GPIO pin (pull down, pullup, etc).
 */
void pulldown(int pin) {
  GPIO[GPPUD] = 1; // enables pull down
  usleep(1);
  GPIO[GPPUDCLK0] = (1 << pin); // uses a clock to set the pin
  usleep(1);
  GPIO[GPPUD] = 0; // takes away control signal
  GPIO[GPPUDCLK0] = 0;
}

/*
 * Description: start of program. Maps the GPIO registers,
 * changes settings of LEDs as outputs and buttons as inputs with pull-down
 * resistors, and runs a 3 led cycle until the stop button is pressed.
 * @return: 0 on normal exit, 1 on permission error.
 * side effects: Requires root privileges, accesses /dev/mem, and controls GPIO.
 * have to use sudo
 */
int main() {
  int fd;

  if (getuid() != 0) { // makes sure the program is run as root
    printf("Run with sudo\n");
    return 1;
  }

  // opens the memory device
  fd = open("/dev/mem", O_RDWR | O_SYNC);

  // maps the gpio to into memory space address
  GPIO = (unsigned int *)mmap(0, getpagesize(), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, GPIO_BASE);

  // configures LEDS as output
  gpio_mode(LED0, 1);
  gpio_mode(LED1, 1);
  gpio_mode(LED2, 1);

  // configures buttons as inputs woth pull down
  gpio_mode(B0, 0);
  pulldown(B0);
  gpio_mode(B1, 0);
  pulldown(B1);
  gpio_mode(B2, 0);
  pulldown(B2);

  int dir = 1; // direction: 1 = left, 0 = right
  int pos = 0; // holds the current LED position

  while (1) {
    if (gpio_read(B0)) // These read the buttons to change directionn and/or
                       // exit
      dir = 0;
    if (gpio_read(B1))
      dir = 1;
    if (gpio_read(B2))
      break;

    gpio_write(
        LED0,
        LED_OFF); // These turn the LEDs off before turning on the next one
    gpio_write(LED1, LED_OFF);
    gpio_write(LED2, LED_OFF);

    if (pos == 0) // these turn on LED on at a time
      gpio_write(LED0, LED_ON);
    if (pos == 1)
      gpio_write(LED1, LED_ON);
    if (pos == 2)
      gpio_write(LED2, LED_ON);

    // This shifts the LED one each loop
    if (dir)
      pos = (pos + 1) % 3;
    else
      pos = (pos + 2) % 3; // equivalent to -1 mod 3

    usleep(SLEEP_TIME); // this controls the time
  }

  gpio_write(LED0, LED_OFF); // This turns the LEDS off when exiting the program
  gpio_write(LED1, LED_OFF);
  gpio_write(LED2, LED_OFF);

  return 0;
}
