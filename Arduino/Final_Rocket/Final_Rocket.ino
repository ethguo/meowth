#define ENCODER_OPTIMIZE_INTERRUPTS

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#ifdef __AVR__
  #include <avr/power.h>
#endif

/* MPU-6050 (GY-521) */
#define OUTPUT_READABLE_YAWPITCHROLL

#define MPU_INTERRUPT_PIN 2


/* NEOPIXELS */
#define INDICATOR_PIN 8
#define INDICATOR_NUMPIXELS 8
#define FIRE_PIN 9
#define FIRE_NUMPIXELS 16

// #define MPU_ADDR 0x68
// #define MPU_MIN_VALUE 265
// #define MPU_MAX_VALUE 402

int staging, parachute, throttle;
double xAngle, yAngle, zAngle;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// int16_t acX, acY, acZ;
// int xComponent, yComponent, zComponent;

Adafruit_NeoPixel indicatorLEDs = Adafruit_NeoPixel(INDICATOR_NUMPIXELS, INDICATOR_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel fireLEDs = Adafruit_NeoPixel(FIRE_NUMPIXELS, FIRE_PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial btSerial(12, 11);
Encoder parachuteEncoder(5, 6);


// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }

void setup() {
  // Serial
  // Serial.begin(9600);
  btSerial.begin(9600);

  // MPU Setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  // pinMode(MPU_INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    // attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    // Serial.println("DMP Initialization success, MPU ready");

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print("DMP Initialization failed (code ");
    // Serial.print(devStatus);
    // Serial.println(")");
  }
  // End MPU Setup
  
  pinMode(4, INPUT);
  pinMode(10, INPUT);
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
  // while (!mpuInterrupt && fifoCount < packetSize) { }
  // while (fifoCount < packetSize) { }

  // reset interrupt flag and get INT_STATUS byte
  // mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    xAngle = ypr[0] * 180/M_PI;
    yAngle = ypr[1] * 180/M_PI;
    zAngle = ypr[2] * 180/M_PI;
  }

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

  // Serial.print(String(staging) + "\t");
  // Serial.print(String(parachute) + "\t");
  // Serial.print(String(throttle) + "\t");
  // Serial.print(String(xAngle) + "\t");
  // Serial.print(String(yAngle) + "\t");
  // Serial.print(String(zAngle) + "\t");
  // Serial.println();

  // delay(1);
}

