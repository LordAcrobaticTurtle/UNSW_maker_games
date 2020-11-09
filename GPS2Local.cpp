#include "GPS2Local.h"

GPS2Local::GPS2Local(){
	
}

GPS2Local::~GPS2Local(){

}

void GPS2Local::init(TinyGPSPlus& gps){
	LatOrigin = gps.location.lat();
	LngOrigin = gps.location.lng();
}

void GPS2Local::computeLocal(TinyGPSPlus& gps){
	double distanceToOrigin = gps.distanceBetween(LatOrigin,LngOrigin,gps.location.lat(),gps.location.lng());
	double angleToOrigin = gps.courseTo(LatOrigin,LngOrigin,gps.location.lat(),gps.location.lng());
	// returns course in degrees (North=0, West=270) from position 1 to position 2
	LocalX = distanceToOrigin*cos(angleToOrigin * PI / 180.0 );
	LocalY = distanceToOrigin*sin(angleToOrigin * PI / 180.0 );
}

double GPS2Local::GetLocalX(){
	return LocalX;
}

double GPS2Local::GetLocalY(){
	return LocalY;
}

void GPS2Local::GPSdisplayRawInfo(TinyGPSPlus& gps){
	Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
