#ifndef _DeadReckonerIMU_h
#define _DeadReckonerIMU_h

#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define UNSIGNED_LONG_MAX 4294967295

class DeadReckoner {

public:
	DeadReckoner();
	void init_timers();
	void compute_velocity(MPU6050& mpu);
	void compute_position(MPU6050& mpu);
	double getX();
	double getY();
	double getHeading();
  	double getvelX();
  	void compute_bias(MPU6050& mpu, int i);
	~DeadReckoner();

private:
	double xc, yc, headingc;
	double velXc;
	double adjustedAccelX;
	double prevAngleX, prevAngleY, prevAnglwZ , prevAccelX, prevAccelY, prevAccelZ;
	double avgbiasX;
	double computeX();
	double computeY();
	unsigned long static getChange(unsigned long current, unsigned long previous);
	double prevVelocityIntegrationTime, prevPositionIntegrationTime;
};

#endif
