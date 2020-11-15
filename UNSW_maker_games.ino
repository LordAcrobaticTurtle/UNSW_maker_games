// UNSW Maker Games 2020 code

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SBUS.h>
#include <Servo.h>
#include <NewPing.h>
#include <MPU6050_light.h>

#include "serial_input_control.h"
#include "DeadReckoner_IMU.h"
#include "GPS2Local.h"
#include "Wire.h"
#include "wheel.h"

#define NUM_EDGE_SENSORS 4
#define MAX_DISTANCE 200 // In cm 

// TX object + accompanying variables
SBUS tx(Serial1);
uint16_t channels[16];
bool failsafe;
bool lostframe; 

// IMU object
MPU6050 mpu(Wire);

unsigned long prevPositionComputeTime = 0;
int panel = 0;
#define MAV_LENGTH 4
int verb = 0;
int timer = 0;
bool stop = 0;

double threshold[4] = {10,10,10,10};
#define time_edge 4

// Variables
wheel WheelFR;
wheel WheelFL;
wheel WheelBL;
wheel WheelBR;


// REAR DRIVER
int ENA_1 = 22;
int ENB_1 = 23;

int IN1_1 = 16;
int IN2_1 = 15;
int IN3_1 = 14;
int IN4_1 = 17;


// FORWARD DRIVER
int ENA_2 = 35;
int ENB_2 = 36;

int IN1_2 = 25;
int IN2_2 = 26;
int IN3_2 = 27;
int IN4_2 = 28;

// Ultrasonic sensors
NewPing SonarFL(37,38 ,MAX_DISTANCE); // Echo = 2, Trig = 3;     
NewPing SonarFR(11,10,MAX_DISTANCE); // Echo = 12, Trig = 11
NewPing SonarBL(21,20,MAX_DISTANCE); // Echo = 20, Trig = 21
NewPing SonarBR(3,2,MAX_DISTANCE); //  Echo = 38, Trig = 37

NewPing Sonar[NUM_EDGE_SENSORS] = {SonarFL, SonarFR, SonarBL, SonarBR};
double Sonar_init[NUM_EDGE_SENSORS];
double Sonar_Prev[NUM_EDGE_SENSORS];
double Sonar_Now[NUM_EDGE_SENSORS];

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	tx.begin();
	
	// IMU setup
	Wire.begin();
	mpu.begin();
	Serial.print(F("Calculating gyro offset, do not move MPU6050"));
	delay(1000);
	mpu.calcGyroOffsets();
	Serial.println("Done!");

	// wheel setup
	WheelFL.setup(IN1_2, IN2_2, ENA_2);
	WheelFL.setLeftTrue(false);
	WheelFR.setup(IN3_2, IN4_2, ENB_2);
	WheelBL.setup(IN3_1, IN4_1, ENB_1);
	WheelBL.setLeftTrue(false);
	WheelBR.setup(IN1_1, IN2_1, ENA_1);
	
	// sonar setup
	Serial.print(F("Getting initial distances"));
	for (int j = 0; j < NUM_EDGE_SENSORS; j++) {
		Sonar_Prev[j] = Sonar[j].ping_cm();
		Sonar_Now[j] = Sonar[j].ping_cm();
	}
	Serial.println("Done!");
	for (int j = 0; j < 4; j++) {
      Serial.print("Initial value" + String( + Sonar_Prev[j]) + ",  ");
    }
}

void loop() {
	// put your main code here, to run repeatedly:
	tx.read(channels, &failsafe, &lostframe);
	
	if (channels[5] > 1500 || 1) {
		if (stop) {
			WheelFL.writeToMotor(250);
			WheelFR.writeToMotor(250);
			WheelBL.writeToMotor(250);
			WheelBR.writeToMotor(250);
		}
		else {
			WheelFL.writeToMotor(500);
			WheelFR.writeToMotor(500);
			WheelBL.writeToMotor(500);
			WheelBR.writeToMotor(500);
		}
		
		if(is_edge_front())
			Serial.println("front edge");
		
		for (int j = 0; j < NUM_EDGE_SENSORS; j++) {
			//Serial.print(String(Sonar[j].ping_cm()) + ",  ");
		}
		
	} else {
		WheelFL.writeToMotor(250);
		WheelFR.writeToMotor(250);
		WheelBL.writeToMotor(250);
		WheelBR.writeToMotor(250);
	}
	
	delay(10);
}

void read_ultrasonic(){
	// read all 4 ultrasonic sensors
	for (int i = 0; i < NUM_EDGE_SENSORS; i++)
		Sonar_Now[i] = Sonar[i].ping_cm();
}

// functions for each sensor; returns 1 with distance gradients chances > threshold
bool edge(int i){
	Sonar_Now[i] = Sonar[i].ping_cm();
	if(abs(Sonar_Now[i]-Sonar_Prev[i]) > threshold[i]) {
		Sonar_Prev[i] = Sonar_Now[i];
		return 1;
	} else
		return 0;
}




bool is_edge_front(){
	if(edge(0) && edge(1))
		return 1;
	else
		return 0;
}

bool is_edge_back(){
return 0;
}

bool is_edge_left(){
return 0;
}

bool is_edge_right(){
return 0;
}
<<<<<<< HEAD


bool right_lean(){
return 0;
}





=======
>>>>>>> 0bd35d8dbcd74e035669d03591744d1d484af785
