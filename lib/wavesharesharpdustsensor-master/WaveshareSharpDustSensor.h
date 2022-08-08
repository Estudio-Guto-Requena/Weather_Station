// WaveshareSharpDustSensor.h

#ifndef _WAVESHARESHARPDUSTSENSOR_h
#define _WAVESHARESHARPDUSTSENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define        COV_RATIO                       0.2            //ug/mmm / mv
#define        NO_DUST_VOLTAGE                 400            //mv

class WaveshareSharpDustSensor{
public:
	WaveshareSharpDustSensor();
	float density;
	float voltage;
	float Conversion(float volts);

};




#endif
