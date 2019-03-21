#include <avr/sleep.h>

enum states {
  stopp,
  start,
  forward,
  reverse,
  brake
};

volatile enum states car_state;
volatile enum states last_state;

// Pins 3 to 14
// Pretty useless enum when using registers directly
enum pins {
  start_pin = 3,
  stop_pin,
  forward_pin,
  reverse_pin,
  brake_pin,
  stop_led,
  start_led,
  motor_dir_cw,
  motor_pwm_pin,
  motor_dir_ccw,
  brake_led,
  pot_pin
};

void setup() {
  Serial.begin(9600);
  // Set pins 3 to 7 to input
  DDRD ^= B11111000;
  // Use pull-up resistor on pins 3 to 7
  PORTD |= B11111000;
  // Set pins 8 to 13 to output
  DDRB |= B00111111;

  car_state = stopp;
}

// Used to wake from sleep
// Can only be used in sleep
void wake() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(start_pin));
  car_state = start;
}

void read_input() {
  // Check pin 4
  if (!(PIND & (1 << PD4))) {
    car_state = stopp;
  } else {
    if (!(PIND & (1 << PD7))) {
      // When entering brake state, save the last state so that we can later return to it
      // Only if last state isn't brake so that it doesn't loop to brake
      if (car_state != brake) {
        last_state = car_state;
      }
      car_state = brake;
    } else if (!(PIND & (1 << PD5))) {
      car_state = forward;
    } else if (!(PIND & (1 << PD6))) {
      car_state = reverse;
    } else if (car_state == brake) {
      // When brake is released return to last state
      car_state = last_state;
    }
  }
}

void spin_fan() {
  analogWrite(motor_pwm_pin, analogRead(pot_pin) / 4);
}

void switch_output() {
  int bit_number;
  // Turn all outputs off
  PORTB &= B11001000;
  if (car_state > forward) {
    bit_number = car_state + 1;
  } else {
    bit_number = car_state;
  }
  // Turn the output, corresponding to the current state, on
  PORTB |= 1 << bit_number;
}

void loop() {
  read_input();
  switch_output();

  switch (car_state) {
    case stopp:
      Serial.println("Stop");
      // Set arduino to sleep
      sleep_enable();
      attachInterrupt(digitalPinToInterrupt(start_pin), wake, LOW); // Start button interrupts sleep
      delay(100);
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      noInterrupts();
      sleep_bod_disable();
      interrupts();
      sleep_cpu();
      sleep_disable();
      break;
    case start:
      Serial.println("Start");
      break;
    case forward:
      Serial.println("Forward");
      spin_fan();
      break;
    case reverse:
      Serial.println("Reverse");
      spin_fan();
      break;
    case brake:
      Serial.println("Brake");
      break;
  }
}
