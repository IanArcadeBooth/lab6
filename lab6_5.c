// lab6_step5_wiringpi.c
// Step 5: WiringPi version (3 LEDs + 3 buttons)
// Uses BCM GPIO numbering via wiringPiSetupGpio().

#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>

// ===== Your LED wiring (PHYSICAL -> BCM) =====
// Pin 11 -> GPIO17
// Pin 12 -> GPIO18
// Pin 13 -> GPIO27
#define LED0 17
#define LED1 18
#define LED2 27

// ===== Buttons (BCM GPIO numbers) =====
// Set these to whatever GPIOs your buttons are wired to.
#define B0 22  // right->left
#define B1 23  // left->right
#define B2 24  // stop/exit

// If your LEDs are wired: 3.3V -> resistor -> LED -> GPIO (GPIO sinks current)
// then LEDs are ACTIVE-LOW.
#define ACTIVE_LOW 0

#if ACTIVE_LOW
#define LED_ON  LOW
#define LED_OFF HIGH
#else
#define LED_ON  HIGH
#define LED_OFF LOW
#endif

static void all_off(void) {
  digitalWrite(LED0, LED_OFF);
  digitalWrite(LED1, LED_OFF);
  digitalWrite(LED2, LED_OFF);
}

static void show_pos(int pos) {
  all_off();
  if (pos == 0)
    digitalWrite(LED0, LED_ON);
  else if (pos == 1)
    digitalWrite(LED1, LED_ON);
  else
    digitalWrite(LED2, LED_ON);
}

int main(void) {
  // BCM numbering (matches GPIO17/18/27 etc.)
  if (wiringPiSetupGpio() != 0) {
    printf("wiringPiSetupGpio failed\n");
    return 1;
  }

  // LEDs outputs
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  all_off();

  // Buttons inputs + pulldown (pressed = HIGH if wired to 3.3V)
  pinMode(B0, INPUT);
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pullUpDnControl(B0, PUD_DOWN);
  pullUpDnControl(B1, PUD_DOWN);
  pullUpDnControl(B2, PUD_DOWN);

  int dir = 1;   // 1 = left->right, 0 = right->left
  int pos = 0;

  while (1) {
    // read buttons (tap to set direction)
    if (digitalRead(B0) == HIGH) { dir = 0; delay(120); } // debounce
    if (digitalRead(B1) == HIGH) { dir = 1; delay(120); } // debounce
    if (digitalRead(B2) == HIGH) break;

    show_pos(pos);

    // move continuously like a marquee
    if (dir) pos = (pos + 1) % 3;
    else     pos = (pos + 2) % 3; // -1 mod 3

    delay(180); // speed (ms) - change to taste
  }

  all_off();
  return 0;
}
