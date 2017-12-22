#ifndef DualMC33926MotorShield_h
#define DualMC33926MotorShield_h

#include "Arduino.h"

class DualMC33926MotorShield
{
	public:
	// default constructor
	DualMC33926MotorShield();
	// custom pins constructor
	DualMC33926MotorShield(uint8_t M1DIR, uint8_t M1PWM, uint8_t M1FB,
	uint8_t M2DIR, uint8_t M2PWM, uint8_t M2FB,
	uint8_t nD2, uint8_t nSF);
	
	void init(); // initialize
	void setM1Velocity(int16_t velocity); // set velocity for motor 1
	void setM2Velocity(int16_t velocity); // set velocity for motor 2
	void setVelocities(int16_t M1Velocity, int16_t M2Velocity); // set velocities for motor 1 and 2
	uint16_t getM1Current(); // get motor 1 current value in mA
	uint16_t getM2Current(); // get motor 2 current value in mA
	uint8_t getFault(); // get fault reading.
	
	private:
	uint8_t nD2;
	uint8_t M1DIR;
	uint8_t M2DIR;
	static const uint8_t M1PWM = 9;
	static const uint8_t M2PWM = 10;
	uint8_t nSF;
	uint8_t M1FB;
	uint8_t M2FB;
};

#endif