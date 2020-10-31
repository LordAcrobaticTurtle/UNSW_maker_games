#include "Wire.h"
#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include "serial_input_control.h"
#include "DeadReckoner_IMU.h"
#include "GPS2Local.h"


// UNSW Maker Games code

unsigned long prevPositionComputeTime = 0;
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds

//static const int RXPin = 50, TXPin = 51;
static const uint32_t GPSBaud = 9600;

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
	Serial.begin(115200);
	for (int i = 2; i <= 13; i++) {
		pinMode(i, OUTPUT); 
	}
	// This configuration makes right turns only 


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
	Serial5.begin(GPSBaud);
	Serial.println(F("Calculating GPS position, do not move"));
	unsigned long start  = millis();
	while (millis() - start < 5000)
		while (Serial5.available() > 0)
			if (gps.encode(Serial5.read()))
				GPS2Local.GPSdisplayRawInfo(gps);
	
	GPS2Local.init(gps);
	
	Serial.println("Done!\n");
  
	// Timers init
	DeadReckoner.init_timers();
}

void loop() {
	// put your main code here, to run repeatedly:
	serial_input_control(Serial);
  
	// IMU Dead Reckoning
	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		mpu.update();
		
		//DeadReckoner.update(mpu);
		DeadReckoner.compute_velocity(mpu);
		DeadReckoner.compute_position(mpu);
		
		Serial.print("IMU X: ");
		Serial.print(String(DeadReckoner.getX()) + " ");
		
		prevPositionComputeTime = millis();
	}
  
	// GPS local positioning
	while (Serial5.available() > 0)
		if (gps.encode(Serial5.read()))
			GPS2Local.GPSdisplayRawInfo(gps);
	
	GPS2Local.computeLocal(gps);

	Serial.print("GPS X: ");
	Serial.println(GPS2Local.GetLocalX());

	delay(20);
}
