#include "DeadReckoner_IMU.h"

DeadReckoner::DeadReckoner():xc(0), yc(0), headingc(0),velXc(0), avgbiasX(0) {};

void DeadReckoner::init_timers(){
	prevVelocityIntegrationTime = micros();
	prevPositionIntegrationTime = micros();
}

void DeadReckoner::compute_bias(MPU6050& mpu, int i){
	avgbiasX = (avgbiasX*i + mpu.getAccX())/(i+1);
}

void DeadReckoner::compute_velocity(MPU6050& mpu){
	double dt = (double)getChange(micros(), prevVelocityIntegrationTime) / 1000000.0; // convert to seconds

	//removing bias
	adjustedAccelX = mpu.getAccX() - avgbiasX;
  
	velXc = dt*(prevAccelX + adjustedAccelX)/2 + velXc;
	
	
	prevAccelX = adjustedAccelX;
	prevVelocityIntegrationTime = micros();
}

void DeadReckoner::compute_position(MPU6050& mpu){
	// xdot = v(t)*cos(theta(t))
    // ydot = v(t)*sin(theta(t))
    // thetadot = omega(t)
	// state vector X = [x; y; heading]
	
	double dt = (double)getChange(micros(), prevPositionIntegrationTime) / 1000000.0; // convert to seconds
	
	xc = xc + dt * velXc * cos(headingc);
  	yc = yc + dt * velXc * sin(headingc);
  	headingc = mpu.getAngleZ();
	
	prevPositionIntegrationTime = micros();
}

double DeadReckoner::getX(){
	return xc;
}

double DeadReckoner::getY(){
	return yc;
}

double DeadReckoner::getHeading(){
	return headingc;
}

double DeadReckoner::getvelX(){
	return velXc;
}

DeadReckoner::~DeadReckoner(){
	
}

unsigned long DeadReckoner::getChange(unsigned long current, unsigned long previous) {
// Overflow has occured
if (current < previous) {
	return UNSIGNED_LONG_MAX - previous + current;
}
// No overflow
return current - previous;
}
