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
double threshold = 1;
int panel = 0;
#define MAV_LENGTH 4
int verb = 0;

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
NewPing SonarFL(3,2 ,MAX_DISTANCE); // Echo = 2, Trig = 3;     
NewPing SonarFR(11,12,MAX_DISTANCE); // Echo = 12, Trig = 11
NewPing SonarBL(21,20,MAX_DISTANCE); // Echo = 20, Trig = 21
NewPing SonarBR(37,38,MAX_DISTANCE); //  Echo = 38, Trig = 37

NewPing Sonar[NUM_EDGE_SENSORS] = {SonarFL, SonarFR, SonarBL, SonarBR};
double Sonar_init[NUM_EDGE_SENSORS];

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
		Sonar_init[j] = Sonar[j].ping_cm();
	}
	Serial.println("Done!");
	
	while(~is_first_panel()) {
		WheelFL.writeToMotor(375);
		WheelFR.writeToMotor(375);
		WheelBL.writeToMotor(375);
		WheelBR.writeToMotor(375);
	}
	WheelFL.writeToMotor(250);
	WheelFR.writeToMotor(250);
	WheelBL.writeToMotor(250);
	WheelBR.writeToMotor(250);
	panel++;
}

int i = 0;
void loop() {
	// put your main code here, to run repeatedly:
	if (millis() - prevPositionComputeTime > 50) { // milliseconds
		mpu.update();
		prevPositionComputeTime = millis();
	}
	WheelFL.writeToMotor(375);
	WheelFR.writeToMotor(375);
	WheelBL.writeToMotor(375);
	WheelBR.writeToMotor(375);
	
	if (is_edge_back())
		panel++;
	
	if (is_edge_front() && panel == MAV_LENGTH && verb == 0) {
		// turn 90
		verb = 1;
	}
	
	if (panel == MAV_LENGTH && verb == 1) {
		// straight
	}
	
	if (is_edge_front() && panel == MAV_LENGTH && verb == 1) {
		// turn 90
		verb = 0;
	}
	
	while (is_edge_left()){
		WheelFL.writeToMotor(375);
		WheelFR.writeToMotor(125);
		WheelBL.writeToMotor(375);
		WheelBR.writeToMotor(125);
	}
	
	while (is_edge_left()){
		WheelFL.writeToMotor(125);
		WheelFR.writeToMotor(375);
		WheelBL.writeToMotor(125);
		WheelBR.writeToMotor(375);
	}
	
	delay(10);
}

bool is_first_panel(){
	if(Sonar[0].ping_cm()-Sonar_init[0] < threshold || Sonar[1].ping_cm()-Sonar_init[1] < threshold)
		return 1;
	else
		return 0;
}

bool is_edge_front(){
	if(Sonar[0].ping_cm()-Sonar_init[0] > threshold || Sonar[1].ping_cm()-Sonar_init[1] > threshold)
		return 1;
	else
		return 0;
}

bool is_edge_back(){
	if(Sonar[2].ping_cm()-Sonar_init[2] > threshold || Sonar[3].ping_cm()-Sonar_init[3] > threshold)
		return 1;
	else
		return 0;
}

bool is_edge_left(){
	if(Sonar[0].ping_cm()-Sonar_init[0] > threshold || Sonar[2].ping_cm()-Sonar_init[2] > threshold)
		return 1;
	else
		return 0;
}

bool is_edge_right(){
	if(Sonar[1].ping_cm()-Sonar_init[1] > threshold || Sonar[3].ping_cm()-Sonar_init[3] > threshold)
		return 1;
	else
		return 0;
}

