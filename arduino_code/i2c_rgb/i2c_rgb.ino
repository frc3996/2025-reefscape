
#include <FastLED.h>
#include <Wire.h>

#define DATA_PIN 5
#define NUM_LEDS 93

#define M 0
#define R 1
#define G 2
#define B 3

#define LED_MODE_SOLID 1
#define LED_MODE_SWIPE 2
#define LED_MODE_BLINK_SLOW 3
#define LED_MODE_BLINK_FAST 4
#define LED_MODE_RAINBOW 5
#define LED_MODE_TREE 6

int wait_time = 0;
int current_mode = LED_MODE_SOLID;
int MRGB_value[4] = {1, 0, 15, 0};
CRGB leds[NUM_LEDS];

int speed = 50;
int height = 100;
unsigned long currentMillis = millis();

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  Wire.begin(0x42);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
}

void receiveEvent(int howMany)
{
  int i=0;
  while(Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    if (i > 3) {
      Serial.print("Received too much data, dropping: ");
      Serial.println(c);
      continue;
    }
    MRGB_value[i++] = c;  
  }
  Serial.print("Received new MRBG value: ");
  Serial.print(MRGB_value[M]);
  Serial.print(", ");
  Serial.print(MRGB_value[R]);
  Serial.print(", ");
  Serial.print(MRGB_value[G]);
  Serial.print(", ");
  Serial.println(MRGB_value[B]);

  if (MRGB_value[M] != 0) {
   current_mode = MRGB_value[M];
  }
}

void loop() {
  if (current_mode == LED_MODE_SOLID) {
    solid();
  } else if (current_mode == LED_MODE_SWIPE) {
    swipe();
  } else if (current_mode == LED_MODE_BLINK_SLOW) {
    slow();
  } else if (current_mode == LED_MODE_BLINK_FAST) {
    fast();
  } else if (current_mode == LED_MODE_RAINBOW) {
    rainbow();
  } else if (current_mode == LED_MODE_TREE) {
    tree();
  } else {
    solid();
  }
}


void solid() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = MRGB_value[R];
    leds[i].g = MRGB_value[G];
    leds[i].b = MRGB_value[B];
  }
  delay(10);
  FastLED.show();
}


void swipe() {
  int swipeTime = 1000; // Swipe time in milliseconds (1 second)
  int delayTime = swipeTime / NUM_LEDS;

  for (int i = NUM_LEDS - 5; i >= 0; i--) {
    // Set color for the color band (adjust as needed)
    leds[i] = CRGB(MRGB_value[R], MRGB_value[G], MRGB_value[B]);
    FastLED.show();
    // delay(delayTime);
  }

  // Turn off the entire color band
  for (int i = NUM_LEDS - 5; i >= 0; i--) {
    leds[i] = CRGB(0, 0, 0);
    FastLED.show();
  }
}

void slow() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = MRGB_value[R];
    leds[i].g = MRGB_value[G];
    leds[i].b = MRGB_value[B];
  }
  delay(500);
  FastLED.show();

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
  delay(500);
  FastLED.show();
}


void fast() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = MRGB_value[R];
    leds[i].g = MRGB_value[G];
    leds[i].b = MRGB_value[B];
  }
  delay(50);
  FastLED.show();

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
  delay(50);
  FastLED.show();
}

void rainbow() {
  //start from red
  for(int colorStep=0; colorStep <= 125; colorStep++) {
    int r = 125;
    int g = 0;
    int b = colorStep;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }

  //into blue
  for(int colorStep=125; colorStep >= 0; colorStep--) {
    int r = colorStep;
    int g = 0;
    int b = 125;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }
  //start from blue
  for(int colorStep=0; colorStep <= 125; colorStep++) {
    int r = 0;
    int g = colorStep;
    int b = 125;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }
  //into green
  for(int colorStep=125; colorStep >= 0; colorStep--) {
    int r = 0;
    int g = 125;
    int b = colorStep;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }
  //start from green
  for(int colorStep=0; colorStep <= 125; colorStep++) {
    int r = colorStep;
    int g = 125;
    int b = 0;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }
  //into yellow
  for(int colorStep=125; colorStep >= 0; colorStep--) {
    int r = 125;
    int g = colorStep;
    int b = 0;
    // Now loop though each of the LEDs and set each one to the current color
    for(int x = 0; x < NUM_LEDS; x++) {
      leds[x] = CRGB(r,g,b);
    }
    // Display the colors we just set on the actual LEDs
    delay(wait_time);
    LEDS.show();
  }
}

void tree() {
    for (int i = 0; i < NUM_LEDS; i++) {
    int position = (i * height) + (currentMillis / speed);
    int brightness = (sin(position * 0.15) + 1) * 127;
    leds[i].r = MRGB_value[R];
    leds[i].g = MRGB_value[G];
    leds[i].b = MRGB_value[B];
    leds[i].subtractFromRGB(255 - brightness);
  }
  
  FastLED.show();
  
  delay(10);


}
