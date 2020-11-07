#ifndef drive_h
#define drive_h

#include <Arduino.h>
#include "DeadReckoner_IMU.h"

class Drive {

public:
	Drive();
	~Drive();
	void motor_control(int frontleft, int frontright, int rearleft, int rearright);
	bool waypoint(double x, double y, int dir, DeadReckoner& DeadReckoner);
	bool rotate(double rotate_angle, DeadReckoner& DeadReckoner);
  bool linear(int distance, int dir);

private:
	double targetX;
	double targetY;
	double remainderX;
	double remainderY;
	double targetHeading;
	
};




#endif