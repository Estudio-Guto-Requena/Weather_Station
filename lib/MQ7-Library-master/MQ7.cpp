#include "Arduino.h"
#include "MQ7.h"

/*
Constructor to initialize the analog pin and input voltage to MQ7

@param pin : Input pin from MQ7 analog pin
@param v_input : Input voltage to MQ7 sensor
*/
MQ7::MQ7(){
}

/*
Function is used to return the ppm value of CO gas concentration
by using the parameter found using the function f(x) = a * ((Rs/R0) ^ b)

@return ppm value of Carbon Monoxide concentration
*/

float MQ7::getPPM(double volts, double ref){
  return (float)(coefficient_A * pow(getRatio(volts, ref), coefficient_B));
}

/*
This function is for the deriving the Rs/R0 to find ppm

@return The value of Rs/R_Load
*/
float MQ7::getRatio(double v_out, double ref){
  return (ref - v_out) / v_out;
}
