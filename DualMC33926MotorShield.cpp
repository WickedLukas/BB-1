#include "DualMC33926MotorShield.h"

// default constructor
DualMC33926MotorShield::DualMC33926MotorShield()
{
	// default pin map
	nD2 = 4;
	M1DIR = 7;
	M2DIR = 8;
	nSF = 12;
	M1FB = A0;
	M2FB = A1;
}

// custom pin map constructor
DualMC33926MotorShield::DualMC33926MotorShield(uint8_t M1DIR_NEW, uint8_t M1PWM_NEW, uint8_t M1FB_NEW,
uint8_t M2DIR_NEW, uint8_t M2PWM_NEW, uint8_t M2FB_NEW,
uint8_t nD2_NEW, uint8_t nSF_NEW)
{
	// remapped pin map (PWM1 and PWM2 cannot be remapped here because the default pins (M1PWM = 9, M2PWM = 10) use timer 1,
	// which is here configured for a PWM frequency of 20kHz)
	nD2 = nD2_NEW;
	M1DIR = M1DIR_NEW;
	M2DIR = M2DIR_NEW;
	nSF = nSF_NEW;
	M1FB = M1FB_NEW;
	M2FB = M2FB_NEW;
}

// initialize
void DualMC33926MotorShield::init()
{
	// define pin modes
	pinMode(M1DIR, OUTPUT);
	pinMode(M1PWM, OUTPUT);
	pinMode(M1FB, INPUT);
	pinMode(M2DIR, OUTPUT);
	pinMode(M2PWM, OUTPUT);
	pinMode(M2FB, INPUT);
	pinMode(nD2, OUTPUT);
	pinMode(nSF, INPUT);
	
	digitalWrite(nD2,HIGH); // nD2 is held high and the PWM is applied to MxPWM (drive-brake mode)

	// configure timer 1 for a PWM frequency of 20kHz
	#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
	// Timer 1 configuration
	// prescaler: clockI/O / 1
	// outputs enabled
	// phase-correct PWM
	// top of 400
	//
	// PWM frequency calculation
	// 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
	TCCR1A = 0b10100000;
	TCCR1B = 0b00010001;
	ICR1 = 400;
	#endif
}

// set velocity for motor 1
void DualMC33926MotorShield::setM1Velocity(int16_t velocity)
{
	uint8_t direction = 0;
	
	// transform signed velocity value into an unsigned velocity value with a direction value
	if (velocity < 0)
	{
		velocity = -velocity;
		direction = 1;
	}
	
	#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
	OCR1A = velocity;
	#else
	analogWrite(M1PWM, velocity);
	#endif
	
	if (direction)
	digitalWrite(M1DIR, HIGH);
	else
	digitalWrite(M1DIR, LOW);
}

// set velocity for motor 2
void DualMC33926MotorShield::setM2Velocity(int16_t velocity)
{
	uint8_t direction = 0;
	
	// transform signed velocity value into an unsigned velocity value with a direction value
	if (velocity < 0)
	{
		velocity = -velocity;
		direction = 1;
	}
	
	#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
	OCR1B = velocity;
	#else
	analogWrite(M2PWM, velocity);
	#endif
	
	if (direction)
	digitalWrite(M2DIR, HIGH);
	else
	digitalWrite(M2DIR, LOW);
}

// set velocity for motor 1 and 2
void DualMC33926MotorShield::setVelocities(int16_t M1Velocity, int16_t M2Velocity)
{
	setM1Velocity(M1Velocity);
	setM2Velocity(M2Velocity);
}

// get motor 1 current value in mA
uint16_t DualMC33926MotorShield::getM1Current()
{
	// 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
	return analogRead(M1FB) * 9;
}

// get motor 2 current value in mA
uint16_t DualMC33926MotorShield::getM2Current()
{
	// 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
	return analogRead(M2FB) * 9;
}

// get error status
uint8_t DualMC33926MotorShield::getFault()
{
	return !digitalRead(nSF);
}
