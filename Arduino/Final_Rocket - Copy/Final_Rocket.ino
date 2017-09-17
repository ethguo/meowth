#define ENCODER_OPTIMIZE_INTERRUPTS

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

/* NEOPIXELS */
#define INDICATOR_PIN 8
#define INDICATOR_NUMPIXELS 8
#define FIRE_PIN 9
#define FIRE_NUMPIXELS 16

/* MPU-6050 (GY-521) */
#define MPU_ADDR 0x68
#define MPU_MIN_VALUE 265
#define MPU_MAX_VALUE 402

int staging, parachute, throttle;
double xAngle, yAngle, zAngle;

int16_t acX, acY, acZ;
int xComponent, yComponent, zComponent;

Adafruit_NeoPixel indicatorLEDs = Adafruit_NeoPixel(INDICATOR_NUMPIXELS, INDICATOR_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel fireLEDs = Adafruit_NeoPixel(FIRE_NUMPIXELS, FIRE_PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial btSerial(12, 11);
Encoder parachuteEncoder(2,3);

void setup() {
  // MPU Setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);
  btSerial.begin(9600);
  
  pinMode(4, INPUT);
  pinMode(10, INPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);

  indicatorLEDs.begin();
  for (int i = 0; i < INDICATOR_NUMPIXELS; i++) indicatorLEDs.setPixelColor(i, 10, 0, 0);
  indicatorLEDs.show();

  fireLEDs.begin();
  for (int i = 0; i < FIRE_NUMPIXELS; i++) fireLEDs.setPixelColor(i, 10, 0, 0);
  fireLEDs.show();
}

void loop() {
  // MPU Read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);

  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();

  xComponent = map(acZ, MPU_MIN_VALUE, MPU_MAX_VALUE, -90, 90);
  yComponent = map(acX, MPU_MIN_VALUE, MPU_MAX_VALUE, -90, 90);
  zComponent = map(acY, MPU_MIN_VALUE, MPU_MAX_VALUE, -90, 90);

  xAngle = atan2(-yComponent, -zComponent) * RAD_TO_DEG + 180;
  yAngle = atan2(-xComponent, -zComponent) * RAD_TO_DEG + 180;
  zAngle = atan2(-yComponent, -xComponent) * RAD_TO_DEG + 180;

  // Parachute Encoder
  parachute = 0;
  if (parachuteEncoder.read() != 0) {
    parachute = 1;
    parachuteEncoder.write(0);
  }

  throttle = analogRead(3);
  staging = digitalRead(4);

  btSerial.print(String(staging) + "\t");
  btSerial.print(String(parachute) + "\t");
  btSerial.print(String(throttle) + "\t");
  btSerial.print(String(xAngle) + "\t");
  btSerial.print(String(yAngle) + "\t");
  btSerial.print(String(zAngle) + "\t");
  btSerial.println();

//  Serial.print(String(staging) + "\t");
//  Serial.print(String(parachute) + "\t");
//  Serial.print(String(throttle) + "\t");
  Serial.print(String(xAngle) + "\t");
  Serial.print(String(yAngle) + "\t");
  Serial.print(String(zAngle) + "\t");
  Serial.println();

  delay(50);
}

