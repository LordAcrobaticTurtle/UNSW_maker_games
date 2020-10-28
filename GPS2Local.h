#ifndef _GPS2Local_h
#define _GPS2Local_h

#include <Arduino.h>
#include <TinyGPS++.h>

class GPS2Local {

public:
	GPS2Local();
	~GPS2Local();
	void init(TinyGPSPlus& gps);
	void GPSdisplayRawInfo(TinyGPSPlus& gps);
	void computeLocal(TinyGPSPlus& gps);
	double GetLocalX();
	double GetLocalY();

private:
	double LatOrigin;
	double LngOrigin;
	double LocalX;
	double LocalY;
};




#endif
