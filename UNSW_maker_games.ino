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

Servo ESC;
Servo brush;

NewPing SonarFL(3,2 ,MAX_DISTANCE); // Echo = 2, Trig = 3;     
NewPing SonarFR(11,10,MAX_DISTANCE); // Echo = 10, Trig = 11
NewPing SonarBL(21,20,MAX_DISTANCE); // Echo = 20, Trig = 21
NewPing SonarBR(37,38,MAX_DISTANCE); //  Echo = 38, Trig = 37

NewPing Sonar[4] = {SonarFL, SonarFR, SonarBL, SonarBR};

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);
  	digitalWrite(LED_BUILTIN, HIGH);
  	tx.begin();
	// IMU setup
	Wire.begin();
	mpu.begin();
	ESC.attach(7);
	brush.attach(6);
  
	Serial.println("Done!");
  	WheelFL.setup(IN1_2, IN2_2, ENA_2);
  	WheelFL.setLeftTrue(false);
	WheelFR.setup(IN3_2, IN4_2, ENB_2);
	WheelBL.setup(IN3_1, IN4_1, ENB_1);
	WheelBL.setLeftTrue(false);
	WheelBR.setup(IN1_1, IN2_1, ENA_1);
	// Timers init
	DeadReckoner.init_timers();
}
int i = 0;
void loop() {
	// put your main code here, to run repeatedly:
	// IMU Dead Reckoning
  tx.read(channels, &failsafe, &lostframe);
  int throttle = channels[2];
  int yaw = channels[0]; 
  int mapped_throttle = map(throttle, 172, 1800, 0,500);
  int mapped_yaw = map(yaw, 182, 1800, -500, 500);
  ESC.writeMicroseconds(map(channels[6], 180, 1800,1000,2000));
  brush.writeMicroseconds(map(channels[7], 180,1800,900,2400));
  mapped_yaw = constrain(mapped_yaw, -250,250);
  
  if (i == 4) { 
    for (int j = 0; j < NUM_EDGE_SENSORS; j++) {
      Serial.print(String(Sonar[j].ping_cm()) + ",  ");
    }
    Serial.println();
    i = 0;
  } else {
    i++;
  }
  
  if (channels[5] > 1500) {
    WheelFL.writeToMotor(mapped_throttle + mapped_yaw);
    WheelFR.writeToMotor(mapped_throttle - mapped_yaw);
    WheelBL.writeToMotor(mapped_throttle + mapped_yaw);
    WheelBR.writeToMotor(mapped_throttle - mapped_yaw);
  } else {
    WheelFL.writeToMotor(250);
    WheelFR.writeToMotor(250);
    WheelBL.writeToMotor(250);
    WheelBR.writeToMotor(250);
  }
  delay(10);
}
