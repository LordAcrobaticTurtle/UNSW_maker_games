// UNSW Maker Games 2020 code


#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SBUS.h>
#include "serial_input_control.h"
#include "DeadReckoner_IMU.h"
#include "GPS2Local.h"
#include "Wire.h"
#include "wheel.h"


// TX object + accompanying variables
SBUS tx(Serial1);
uint16_t channels[16];
bool failsafe;
bool lostframe; 

// IMU object
MPU6050 mpu(Wire);

// DeadReckoner IMU object
DeadReckoner DeadReckoner;

// The TinyGPS++ object
TinyGPSPlus gps;

// GPS to local object
GPS2Local GPS2Local;

// Variables
unsigned long prevPositionComputeTime = 0;

wheel WheelFR;
wheel WheelFL;
wheel WheelBL;
wheel WheelBR;


int ENA_1 = 22;
int ENB_1 = 23;

// REAR DRIVER
int IN1_1 = 16;
int IN2_1 = 15;
int IN3_1 = 14;
int IN4_1 = 17;

int ENA_2 = 35;
int ENB_2 = 36;

// FORWARD DRIVER
int IN1_2 = 25;
int IN2_2 = 26;
int IN3_2 = 27;
int IN4_2 = 28;


void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	
  tx.begin();
	// IMU setup
	Wire.begin();
	mpu.begin();
	Serial.print(F("Calculating gyro offset, do not move MPU6050"));
	delay(1000);
	mpu.calcGyroOffsets();
	Serial.println("Done!");

	// Calculating IMU bias
	Serial.print(F("Calculating bias, do not move MPU6050"));
	for (int i = 0; i<200; i++) {
		mpu.update();
		DeadReckoner.compute_bias(mpu,i);
  
		delay(20);
	}
	Serial.println("Done!");
	
	// GPS setup
	Serial5.begin(9600);
	Serial.print(F("Calculating GPS position, do not move"));
	//unsigned long start  = millis();
	//while (millis() - start < 5000)
	do {
		while (Serial5.available() > 0)
			if (gps.encode(Serial5.read()))
				GPS2Local.GPSdisplayRawInfo(gps);
	}
	while(~gps.location.isUpdated());
	
	GPS2Local.init(gps);
	
	Serial.println("Done!");
  WheelFL.setup(IN1_1, IN2_1, ENA_1);
  WheelFL.setup(IN3_1, IN4_1, ENB_1);
  WheelFL.setup(IN1_2, IN2_2, ENA_2);
  WheelFL.setup(IN3_2, IN4_2, ENA_2);
	// Timers init
	DeadReckoner.init_timers();
}

void loop() {
	// put your main code here, to run repeatedly:
  
	// IMU Dead Reckoning
	if (millis() - prevPositionComputeTime > 50) {	// milliseconds
		mpu.update();
		
		DeadReckoner.compute_velocity(mpu);
		DeadReckoner.compute_position(mpu);
		
		Serial.print("IMU X: "); Serial.print(DeadReckoner.getX()); Serial.print("\t");
		Serial.print("IMU Y: "); Serial.print(DeadReckoner.getY()); Serial.print("\t");
		Serial.print("IMU Heading: "); Serial.print(DeadReckoner.getHeading()); Serial.print("\t");
		
		prevPositionComputeTime = millis();
	}
  
	// GPS local positioning
	while (Serial5.available() > 0)
		if (gps.encode(Serial5.read()))
			//GPS2Local.GPSdisplayRawInfo(gps);
	
	if (gps.location.isUpdated()) {
		GPS2Local.computeLocal(gps);
		Serial.print("GPS X: "); Serial.print(GPS2Local.GetLocalX()); Serial.print("\t");
		Serial.print("GPS Y: "); Serial.print(GPS2Local.GetLocalY()); Serial.print("\t");
	}
	Serial.println();

  WheelFR.writeToMotor(0);
	delay(20);
}
