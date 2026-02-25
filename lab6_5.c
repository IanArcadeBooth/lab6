/*
 * File: lab6_5.c
 * Author: Ian Booth
 * Date: 2026/02/25
 * Description: Controls a 3-LED marquee on a Raspberry Pi using WiringPi.
 * Three buttons control direction and program exit.
 */

#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>

#define LED0 17 // physical pin11
#define LED1 18 // physcial pine12
#define LED2 27 // physical pin13

#define B0 22 //(phyysical pin15) right
#define B1 23 //(physical pin16) left
#define B2 24 //(physical pin 18) stop/exit

#define WAIT_TIME 180
#define ACTIVE_LOW 0

#if ACTIVE_LOW // using this if to figure out why it seemed like my LEDs were on
#define LED_ON LOW // two at a time
#define LED_OFF HIGH
#else
#define LED_ON HIGH
#define LED_OFF LOW
#endif

/*
 * Description: Turns all LEDs off.
 * @return: none
 * side effects: Sets all LED GPIO pins to the off state.
 */
static void all_off(void) {
  digitalWrite(LED0, LED_OFF);
  digitalWrite(LED1, LED_OFF);
  digitalWrite(LED2, LED_OFF);
}

/*
 * Description: turns on one lED and turns off the others.
 * @param pos: led to turn on (0 = LED0, 1 = LED1, 2 = LED2).
 * @return: none
 * side effects: Turns on/off LEDS?? (not sure if this is a side effect
 */
static void show_pos(int pos) {
  all_off();
  if (pos == 0)
    digitalWrite(LED0, LED_ON);
  else if (pos == 1)
    digitalWrite(LED1, LED_ON);
  else
    digitalWrite(LED2, LED_ON);
}

/*
 * Description: start of program. starts WiringPi, configures the GPIO
 * pins for LEDs and buttons, and runs a 3 led cycle until the stop
 * button is pressed.
 * @return: 0 on normal exit, non-zero on error.
 * side effects: Controls GPIO output pins and reads GPIO input pins.
 */
int main(void) {
  if (wiringPiSetupGpio() !=
      0) { // this sets the defined LED pins as the braodcom (BCM) number
    printf("wiringPiSetupGpio failed\n"); // of the pins
    return 1;
  }

  pinMode(LED0, OUTPUT); // sets pins as output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  all_off();

  pinMode(B0, INPUT); // makes buttons inputs and pulldowns
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pullUpDnControl(B0, PUD_DOWN);
  pullUpDnControl(B1, PUD_DOWN);
  pullUpDnControl(B2, PUD_DOWN);

  int dir = 1; // 1 is right, 0 is left
  int pos = 0; // holds the currents position of LED that is on

  while (1) {
    if (digitalRead(B0) == HIGH) {
      dir = 0;
      delay(120);
    } // these read the buttons and debounce
    if (digitalRead(B1) == HIGH) {
      dir = 1;
      delay(120);
    } // with small delay
    if (digitalRead(B2) == HIGH)
      break; // no delay for deabounce beacuse the program is now done

    show_pos(pos);

    if (dir)
      pos = (pos + 1) % 3; // moves "forward" through cycle
    else
      pos = (pos + 2) % 3; // moves "backwards" through cycle

    delay(WAIT_TIME); // speed of the position change/loop
  }

  all_off();
  return 0;
}
