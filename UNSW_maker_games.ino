#include "Wire.h"
#include <MPU6050_light.h>
#include "DeadReckoner_IMU.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "GPS2Local.h"

// UNSW Maker Games code

// pins 2,3,4,5,6,7 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on the rear motor driver
// 
// pins 8,9,10,11,12,13 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on forward motor driver
// IN4 and IN3 control the direction of the left side motors (respective to the raspi camera)
// IN1 and IN2 control the right side motors. 
// All above pins are PWM on the Arduino Mega
// Run the cam_test.py from the desktop using python3 to see through the raspi camera
// 

// It is possible to set up communication between the pi and the arduino.Receiving strings from arduino is available under "serial_test.py" 
// RUn it wihle the arduino is using "serial.print". ENSURE THE SERIAL MONITOR IS NOT OPEN
// https://www.jaycar.com.au/arduino-compatible-stepper-motor-controller-module/p/XC4492 These are the motors drivers being used. The specifications
// tab has datasheets on how to use. 
// An MPU6050 is connected on the I2C bus. It can output accel + gyro data. 
/*
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH); 
 */



int ENB_1 = 2;
int IN1_1 = 6;
int IN2_1 = 5;
int IN3_1 = 4;
int IN4_1 = 3;
int ENA_1 = 7;


int ENA_2 = 13;
int IN1_2 = 12;
int IN2_2 = 11;
int IN3_2 = 10;
int IN4_2 = 9;
int ENB_2 = 8;

int val = 0;

unsigned long prevPositionComputeTime = 0;
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds

//static const int RXPin = 50, TXPin = 51;
static const uint32_t GPSBaud = 115200;

// IMU object
MPU6050 mpu(Wire);

// DeadReckoner IMU object
DeadReckoner DeadReckoner;

// The TinyGPS++ object
TinyGPSPlus gps;

// GPS to local object
GPS2Local GPS2Local;

// The software serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	for (int i = 2; i <= 13; i++) {
		pinMode(i, OUTPUT); 
	}
	// This configuration makes right turns only 

	digitalWrite(IN1_1, LOW);
	digitalWrite(IN2_1, HIGH);
	digitalWrite(IN3_1, LOW);
	digitalWrite(IN4_1, HIGH);

	digitalWrite(IN1_2, HIGH);
	digitalWrite(IN2_2, LOW);
	digitalWrite(IN3_2, LOW);
	digitalWrite(IN4_2, HIGH); 
	Serial.println("WAITING ON INPUT");

	// IMU setup
	Wire.begin();
	mpu.begin();
	Serial.println(F("Calculating gyro offset, do not move MPU6050"));
	delay(1000);
	mpu.calcGyroOffsets();
	Serial.println("Done!\n");

	// Calculating IMU bias
	Serial.println(F("Calculating bias, do not move MPU6050"));
	for (int i = 0; i<200; i++) {
		mpu.update();
		//DeadReckoner.update(mpu);
		DeadReckoner.compute_bias(mpu,i);
  
		delay(20);
	}
	Serial.println("Done!\n");
	
	// GPS setup
	Serial2.begin(GPSBaud);
	Serial.println(F("Calculating GPS position, do not move"));
	while (Serial2.available() > 0)
		if (gps.encode(Serial2.read()))
			GPS2Local.GPSdisplayRawInfo(gps);

	if (millis() > 5000 && gps.charsProcessed() < 10)
	{
		Serial.println(F("No GPS detected: check wiring."));
		while(true);
	}
	
	GPS2Local.init(gps);
	
	Serial.println("Done!\n");
  
	// Timers init
	DeadReckoner.init_timers();
}

void loop() {
	// put your main code here, to run repeatedly:
  
	serial_input();
  
	// IMU Dead Reckoning
	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		mpu.update();
		
		//DeadReckoner.update(mpu);
		DeadReckoner.compute_velocity(mpu);
		DeadReckoner.compute_position(mpu);
		
		Serial.print("IMU X: ");
		Serial.println(DeadReckoner.getX());
		
		prevPositionComputeTime = millis();
	}
  
	// GPS local positioning
	while (Serial2.available() > 0)
		if (gps.encode(Serial2.read()))
			GPS2Local.GPSdisplayRawInfo(gps);

	if (millis() > 5000 && gps.charsProcessed() < 10)
	{
		Serial.println(F("No GPS detected: check wiring."));
		while(true);
	}
	
	GPS2Local.computeLocal(gps);

	Serial.print("GPS X: ");
	Serial.println(GPS2Local.GetLocalX());
  
  
	delay(20);
}















void serial_input() {
	if (Serial.available()) {
		char bite = Serial.read();
    if (bite == 'g') {
		Serial.println("GO");
		val+= 10;
		analogWrite(ENA_1, val);
		analogWrite(ENB_1, val);
		analogWrite(ENA_2, val);
		analogWrite(ENB_2, val);
    }
    if (bite == 's') {
		Serial.println("STOP");
		val-= 10;
		analogWrite(ENA_1, val);
		analogWrite(ENB_1, val);
		analogWrite(ENA_2, val);
		analogWrite(ENB_2, val);
  
    }
    if (bite == 'q') {
		analogWrite(ENA_1, 0);
		analogWrite(ENB_1, 0);
		analogWrite(ENA_2, 0);
		analogWrite(ENB_2, 0);
    }
    if (bite == 'a') {
		digitalWrite(IN1_1, HIGH);
		digitalWrite(IN2_1, LOW);
		digitalWrite(IN3_1, HIGH);
		digitalWrite(IN4_1, LOW);

		digitalWrite(IN1_2, LOW);
		digitalWrite(IN2_2, HIGH);
		digitalWrite(IN3_2, HIGH);
		digitalWrite(IN4_2, LOW);
    }
    if (bite == 'd') {
		digitalWrite(IN1_1, LOW);
		digitalWrite(IN2_1, HIGH);
		digitalWrite(IN3_1, LOW);
		digitalWrite(IN4_1, HIGH);

		digitalWrite(IN1_2, HIGH);
		digitalWrite(IN2_2, LOW);
		digitalWrite(IN3_2, LOW);
		digitalWrite(IN4_2, HIGH);
    }
    if (bite == 'f') {
		analogWrite(ENA_1, 255);
		analogWrite(ENB_1, 255);
		analogWrite(ENA_2, 255);
		analogWrite(ENB_2, 255);
    }
    if (bite == 'w') {
		digitalWrite(IN1_1, HIGH);
		digitalWrite(IN2_1, LOW);
		digitalWrite(IN3_1, LOW);
		digitalWrite(IN4_1, HIGH);

		digitalWrite(IN1_2, LOW);
		digitalWrite(IN2_2, HIGH);
		digitalWrite(IN3_2, LOW);
		digitalWrite(IN4_2, HIGH);
    }
  }
}
