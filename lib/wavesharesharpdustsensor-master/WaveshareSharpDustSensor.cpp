//
//
//

#include "WaveshareSharpDustSensor.h"

float WaveshareSharpDustSensor::Conversion(float volts) {
	volts *= 1000.;
	/*
	voltage to density
	*/
	Serial.println(volts);
	if (voltage >= NO_DUST_VOLTAGE){
		voltage -= NO_DUST_VOLTAGE;
		density = voltage * COV_RATIO;
	}
	else density = 0;
	return density;
}

WaveshareSharpDustSensor::WaveshareSharpDustSensor()
{
}
