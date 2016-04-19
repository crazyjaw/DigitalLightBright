#include "avr/interrupt.h";
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <math.h>
#include <avr/sleep.h>

#define LED_PIN 0 // neopixel data pin
#define ENCODER_PIN_1 3
#define ENCODER_PIN_2 4
#define STEP_SIZE 1.4 // how far to step through the color wheel for each click of your encoder
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

volatile float value = -STEP_SIZE;  // position on the color wheel
volatile int lastEncoded = 0;
volatile bool isOff = true;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  digitalWrite(ENCODER_PIN_1, HIGH);
  digitalWrite(ENCODER_PIN_2, HIGH);

  adc_disable(); // ADC uses ~320uA
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  GIMSK = 0b00100000;       // Enable pin change interrupts
  PCMSK =  0b00011000;      // Enable pin change interrupt for PB3 and PB4
  sei();                    // Turn on interrupts
}

void loop() {
  rainbow(value);
}

void rainbow(int pos) {
  for (int i = 0; i < strip.numPixels(); i++) {
    if ( pos < 0) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    } else {
      int value = (int) round(pos);
      strip.setPixelColor(i, Wheel(value));
    }
  }
  strip.show();
  sleep_enable();
  sleep_cpu();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// This is the ISR that is called on each interrupt
// Taken from http://bildr.org/2012/08/rotary-encoder-arduino/
ISR(PCINT0_vect) {
  int MSB = digitalRead(ENCODER_PIN_1); //MSB = most significant bit
  int LSB = digitalRead(ENCODER_PIN_2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    value += STEP_SIZE;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    value -= STEP_SIZE;

  lastEncoded = encoded; //store this value for next time

  if (value <  -(STEP_SIZE * 4)) {
    value = 255;
  }
  if (value > 255) {
    value =  -(STEP_SIZE * 4);
  }

}


