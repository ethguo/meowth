
/*
 * Credit to: 
 *    http://www.pitt.edu/~mpd41/Angle.ino
 *    
 * 
 * MPU6050 to Arduino Compementary Angle Finder
 * 
 * This sketch calculates the angle of an MPU6050 relative to the ground using a complementary filter.
 * It is designed to be easy to understand and easy to use. 
 * 
 * The following are your pin connections for a GY-521 breakout board to Arduino UNO or NANO:
 * MPU6050  UNO/NANO
 * VCC      +5v
 * GND      GND (duh)
 * SCL      A5
 * SDA      A4
 * 
 * Of note: this sketch uses the "I2C Protocol," which allows the Arduino to access multiple pieces of data
 * on the MPU6050 with only two pins.  Way cooler than needing 7 pins for the 7 pieces of data you can get
 * from your MPU6050.  The two wires are a data wire and a clock wire. The "Wire.h" library does most of this 
 * communication between the Arduino and the MPU6050. It requires that the Arduino use the A5 adn A4 pins.
 * Other Arduinos (Arduini?) may use different pins for the I2C protocol, so look it up on the Arduino.cc website. 
 * The I2C protocol works this way like a conversation between the Arduino and the MPU.  It goes something like this:
 * Arduino: "Hey 0x68." (0x68 is the MPU's address)
 * MPU6050: "wat"
 * Arduino: "Gimme yer x acceleration data." (except Arduino is a computer, so it calls that data "0x3B"
 * MPU6050: "Ugh fine. 16980"
 * Arduino: "K stop now"
 * MPU6050: "K"
 * 
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <SoftwareSerial.h>
#include<Wire.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN 8
#define NUMPIXELS  8
int thrust, thrustold = 0, paratrig = 0, x = 0, pos;
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY, compAngleZ, compAngleXo, compAngleYo, compAngleZo; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
SoftwareSerial softSerial(12, 11);
Encoder para(2,3);

void setup() {
  // Set up MPU 6050:
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  softSerial.begin(9600);
  Serial.begin(9600);
  delay(100);

  //setup starting angle
  //1) collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;
  

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  //start a timer
  timer = micros();
  pinMode(4, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  strip.begin();
  strip.show();
}

void loop() {
//Now begins the main loop. 
  //Collect raw data from the sensor.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  
  //W00T print dat shit out, yo!
  if(x<250) {
    compAngleXo = compAngleX;
    compAngleYo = compAngleY;
    x++;
  }
  else {
    digitalWrite(13, HIGH);
    for(int i=0;i<NUMPIXELS;i++){
      strip.setPixelColor(i, strip.Color(50,0,0));
      strip.show();
    }
  }

  pos = para.read();
  
  if(pos != 0) {
    paratrig = 1;
    para.write(0);
  }
  else {
    paratrig = 0;
  }
  
  thrust = analogRead(3);

  thrustold = thrust;
 //Bt Serial
  softSerial.print(digitalRead(4)); softSerial.print("\t"); //Flame Sensor
  softSerial.print(paratrig); softSerial.print("\t"); //Pullcord
  softSerial.print(compAngleX-compAngleXo); softSerial.print("\t");
  softSerial.print(compAngleY-compAngleYo); softSerial.print("\t");
  softSerial.print(thrust); softSerial.print("\t"); //Softpot
  //softSerial.print(analogRead(6)); //Flame analog
  softSerial.println();

  //Normal Serial
  Serial.print(digitalRead(4)); Serial.print("\t"); //Flame Sensor
  Serial.print(paratrig); Serial.print("\t"); //Pullcord
  Serial.print(compAngleX-compAngleXo); Serial.print("\t");
  Serial.print(compAngleY-compAngleYo); Serial.print("\t");
  Serial.print(thrust); Serial.print("\t"); //Softpot
  //Serial.print(analogRead(6)); //Flame analog
  Serial.println();
  delay(25);
  
}
