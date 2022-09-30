/*
* BB-1.ino
*
* Created: 05.11.2016
* Author: Lukas
*
* Using I2C device class (I2Cdev) and MPU6050 class by Jeff Rowberg (Arduino)
*
*/

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "DualMC33926MotorShield.h"
#include "hd44780.h"
#include "hd44780ioClass/hd44780_I2Cexp.h"

#include "ema_filter.h"
#include "kalman_filter.h"
#include "PID_controller.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

hd44780_I2Cexp lcd(0x27);

// address of the Arduino slave reading encoder values
#define SLAVE_ADDRESS 0x08

// flag to start initialization
boolean refresh = true;

// initialization flag, necessary to calculate starting values
boolean first;

// specifies timeout for bluetooth communication with BB-1 remote in microseconds
const int32_t REMOTE_TIMEOUT = 100000;

// maximum number of chars sent in one serial message
const byte MAX_CHARS = 32;

// factor for converting a radian number to an equivalent number in degrees
const float RAD2DEG = (float) 4068 / 71;

// acceleration of gravity
const float G = 9811;

// configuration for digital low pass filter
const char DLPF_MODE = MPU6050_DLPF_BW_188;

// configuration for an exponential moving average filter used to filter the BB-1 velocity, calculated from the wheel encoder values
const float EMA_ALPHA_VELOCITY = 0.03;	// 0.03, 0.04

// configuration for an exponential moving average filter used to filter the BB-1 delta velocity, calculated from the wheel encoder values
const float EMA_ALPHA_DELTAVELOCITY = 0.3;	// 0.3, 0.4

// configuration for an exponential moving average filter used to filter the velocity setpoint received from the BB-1 remote
const float EMA_ALPHA_VELOCITY_SP = 0.01;	// 0.01

// configuration for an exponential moving average filter used to filter the delta velocity setpoint received from the BB-1 remote
const float EMA_ALPHA_DELTAVELOCITY_SP = 0.1; // 0.1

// sample rate = gyroscope output rate / (1 + SAMPLE_RATE_DIVIDER)
// gyroscope output rate is 8kHz for DLPF_MODE MPU6050_DLPF_BW_256 else 1 kHz
const char SAMPLE_RATE_DIVIDER = 6;

// targeted MPU update time in microseconds
int32_t mpu_update_time;

// measured MPU update time in microseconds
int32_t dt = 0;
// measured MPU update time in seconds
float dT = 0;

// range of accelerometer and gyro
const int16_t ACCEL_RANGE = MPU6050_ACCEL_FS_8;
const int16_t GYRO_RANGE = MPU6050_GYRO_FS_1000;

// acceleration of gravity in LSB
const int16_t G_LSB = pow(2, 16) / (4 * pow(2, ACCEL_RANGE));

// MPU sensitivity
const float ACCEL_SENS = pow(2, 16) / (4 * pow(2, ACCEL_RANGE)) / G;
const float GYRO_SENS = pow(2, 16) / (500 * pow(2, GYRO_RANGE));

// maximum motor RPM
const int16_t MOTOR_RPM = 350;

// encoder resolution in counts per revolution
const int16_t ENCODER_RESOLUTION = 960;

// tire diameter in mm
const int16_t TIRE_DIAMETER = 120;

// wheelbase in mm
//const int16_t WHEELBASE = 235;

// maximum angle of BB-1 (failsafe)
const float ANGLE_MAX = 55;

// maximum allowed angle for BB-1
const float ANGLE_LIMIT = 45;

// maximum velocity of BB-1
const float VELOCITY_MAX = PI * MOTOR_RPM * TIRE_DIAMETER / 60;

// maximum allowed velocity for BB-1
const float VELOCITY_LIMIT = 0.5 * VELOCITY_MAX;

// maximum allowed delta velocity for BB-1
const float DELTA_VELOCITY_LIMIT = 0.25 * VELOCITY_LIMIT;

// MPU raw measurements
int16_t ax, ay, az;
int16_t gx, gy, gz;
// last MPU raw measurements
int16_t ax0, ay0, az0;
int16_t gx0, gy0, gz0;

// encoder raw measurements
int8_t enc_count_M1, enc_count_M2;
// ultrasonic measurements in cm
uint8_t front_distance, rear_distance;

// Kalman filter class
// parameters: qp_angle, qp_rate, qp_rateBias, r_acc, r_gyro, angle
// r_acc can maybe be reduced, to speed up rateBias and angle offset approximation, if angle_x is used for Kalman filter
KalmanFilter kalmanFilter_x(10000, 10000000000, 0.0000000001, 250000, 0.00000001, 0);	// for dT = 0.07s; (10000, 10000000000, 0.0000000001, 250000, 0.00000001, 0);

// PID values for angle controller
float P_angle = 10;		// 10, 11, 8
float I_angle = 0;		// 0, 0, 0
float D_angle = 0.4;	// 0.4, 0.44, 0.4

// PID values for velocity controller
float P_velocity = 0.044;	// 0.044, 0.04, 0.04
float I_velocity = 0;		// 0, 0, 0.03
float D_velocity = 0;		// 0, 0, 0

// PID values for delta velocity controller
float P_deltaVelocity = 0.7;	// 0.7
float I_deltaVelocity = 3;		// 3
float D_deltaVelocity = 0.005;	// 0.005 tested with EMA_ALPHA_VELOCITY = 0.4

// PID controller classes for angle (mpu) and velocity (encoder)
PID_controller pid_angle_x(P_angle, I_angle, D_angle, 255, 255);
PID_controller pid_velocity_y(P_velocity, I_velocity, D_velocity, ANGLE_LIMIT, ANGLE_LIMIT);
PID_controller pid_deltaVelocity_y(P_deltaVelocity, I_deltaVelocity, D_deltaVelocity, 255, 255);

// motor controller class
DualMC33926MotorShield md(11, 9, A0, 8, 10, A1, 4, 12);	// remap M1DIR from pin 7 to pin 11

//#define PERFORM_MPU_CALIBRATION	// calibrate the MPU

#define MPU_INTERRUPT_PIN 7

// potentiometer pins to control PID values
#define P_PIN A3
#define I_PIN A4
#define D_PIN A5

// orders for communication
#define HELLO_BB1 'B'
#define HELLO_BB1_REMOTE 'R'
#define VELOCITY 'V'
#define CONTROL 'C'

bool blinkState = false;

// MPU interrupt status byte
uint8_t mpuIntStatus;

// update from BB-1 remote
void remote_update(int16_t& velocity_sp, int16_t& deltaVelocity_sp);

// get data from BB-1 remote
boolean getData(char order, char* receivedChars, boolean& remoteConnection, boolean& requestRemoteData);

// update velocity setpoints
void velocity_sp_update(char *receivedChars, int16_t& velocity_sp, int16_t& deltaVelocity_sp);

// update accel, gyro and encoder values and measure update time
void sensor_update();

// calculate velocities
void calc_velocities(float& velocity_M1, float& velocity_M2);

// calculate accel x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel);

// calculate gyro x, y and z angles in degrees (not necessary for filters)
void calc_gyroAngles(float& angle_x_gyro, float& angle_y_gyro, float& angle_z_gyro);

// calculate x angle of BB1 in degrees, which, compared to accel x angle, is corrected by the error caused by BB-1 acceleration
void calc_angleX(float angle_x_accel, float velocity, float& angle_x);

// print display (mode 1: PID; mode 2: average velocity, angle, dt; mode 3: average velocity, angle, minimal distance)
void printDisplay(boolean refresh, int8_t mode, float velocity, float angle_x_KF);

// calibrate MPU
void calibMotion6(int16_t num, int16_t accuracy_accel, int16_t accuracy_gyro);

// calculate mean MPU measurements
void meanMotion6(int16_t num, int16_t& mean_ax, int16_t& mean_ay, int16_t& mean_az, int16_t& mean_gx, int16_t& mean_gy, int16_t& mean_gz);

// send serial data to Processing
void sendSerial(float angle_x_gyro, float angle_y_gyro, float angle_z_gyro, float angle_x_accel, float angle_y_accel, float angle_x_CF, float angle_y_CF, float angle_x_KF, float angle_y_KF);


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dataReady() {
	mpuInterrupt = true;
}

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


void setup() {
	// configure Arduino interrupt pin for MPU
	pinMode(MPU_INTERRUPT_PIN, INPUT);

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif

	#ifdef DEBUG
	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo eNUMeration, others continue immediately
	#endif
	
	// initialize serial1 communication for BB-1 remote through HC-05 bluetooth modules
	Serial1.begin(115200);
	while (!Serial1); // wait for Leonardo eNUMeration, others continue immediately

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Arduino
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize I2C devices
	DEBUG_PRINTLN(F("\nInitializing I2C devices..."));
	
	// initialize LCD (initializing the LCD seems to negatively affect interrupts and even more does actual printing)
	lcd.begin(20, 4);
	
	// initialize MPU
	mpu.initialize();
	mpu.setDLPFMode(DLPF_MODE);
	mpu.setRate(SAMPLE_RATE_DIVIDER);
	mpu.setFullScaleAccelRange(ACCEL_RANGE);
	mpu.setFullScaleGyroRange(GYRO_RANGE);
	mpu.setIntEnabled(0x01);

	// verify connection
	DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful.") : F("MPU6050 connection failed."));

	// enable Arduino interrupt detection
	DEBUG_PRINTLN(F("\nEnabling interrupt detection...\n"));
	attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dataReady, RISING);

	// initialize Dual Motor Shield
	md.init();

	// Calculate targeted MPU update time:
	// MPU update time = 1 / MPU sample rate = (SAMPLE_RATE_DIVIDER + 1) / gyroscope output rate
	// The gyroscope output rate depends on the Low Pass Filter used (DLPF_MODE)
	if (DLPF_MODE == MPU6050_DLPF_BW_256) {	// gyroscope output rate = 8 kHz
		mpu_update_time = (SAMPLE_RATE_DIVIDER + 1) * 125;	// mpu_update_time in microseconds, 1000000 / 8000 = 125
	}
	else {	// gyroscope output rate = 1 kHz
		mpu_update_time = (SAMPLE_RATE_DIVIDER + 1) * 1000;	// mpu_update_time in microseconds, 1000000 / 1000 = 1000
	}
	
	#ifdef PERFORM_MPU_CALIBRATION
	// calibration parameter
	const int16_t NUM = 500;
	const int16_t ACCURACY_ACCEL = 9;
	const int16_t ACCURACY_GYRO = 4;
	
	// calibrate MPU
	DEBUG_PRINTLN(F("\nCalibrating ..."));
	calibMotion6(NUM, ACCURACY_ACCEL, ACCURACY_GYRO);
	DEBUG_PRINTLN(F("Calibration completed.\n"));
	#else
	// use offsets from previous calibration
	mpu.setXAccelOffset(-450);  //  -449, -459
	mpu.setYAccelOffset(2550);  //  2496, 2591
	mpu.setZAccelOffset(1278);  //  1241, 1216

	mpu.setXGyroOffset(16); //  20, 29
	mpu.setYGyroOffset(41); //  29, 28
	mpu.setZGyroOffset(23); //  29, 28
	#endif
}


void loop() {
	// set first if refresh has fired to start initialization
	if (refresh) {
		refresh = false;
		first = true;
	}
	else {
		first = false;
	}
	
	// update accel, gyro and encoder values and measure update time
	sensor_update();
	
	// motor 1 and motor 2 velocities
	static float velocity_M1, velocity_M2;
	// calculate velocities from encoder data
	calc_velocities(velocity_M1, velocity_M2);

	// average velocity
	static float velocity;
	velocity = (velocity_M1 + velocity_M2) * 0.5;
	
	// EMA filtered average velocity
	static float velocity_filtered = velocity;
	// filter average velocity
	velocity_filtered = ema_filter(velocity, velocity_filtered, EMA_ALPHA_VELOCITY);

	// delta velocity
	static float deltaVelocity;
	deltaVelocity = (velocity_M1 - velocity_M2) * 0.5;
	
	// EMA filtered delta velocity
	static float deltaVelocity_filtered = deltaVelocity;
	// filter delta velocity
	deltaVelocity_filtered = ema_filter(deltaVelocity, deltaVelocity_filtered, EMA_ALPHA_DELTAVELOCITY);
	
	// accel angles
	static float angle_x_accel, angle_y_accel;
	// calculate accel x and y angles in degrees
	calc_accelAngles(angle_x_accel, angle_y_accel);
	
	// gyro angles
	static float angle_x_gyro = 0, angle_y_gyro = 0, angle_z_gyro = 90;
	// calculate gyro x, y and z angles in degrees (not necessary for filters)
	calc_gyroAngles(angle_x_gyro, angle_y_gyro, angle_z_gyro);
	
	// angle x of BB1
	static float angle_x;
	// calculate x angle of BB1 in degrees, which, compared to accel x angle, is corrected by the error caused by BB-1 acceleration
	calc_angleX(angle_x_accel, velocity_filtered, angle_x);
	
	/*
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// COMPLEMENTARY FILTER

	// complementary filter time constant
	static const float C_FILTER_T = 0.3;
	
	// delta gyro angles
	static float delta_angle_x_gyro, delta_angle_y_gyro;
	
	// complementary filtered angles
	static float angle_x_CF = 0, angle_y_CF = 0;
	
	// delta gyro angles
	delta_angle_x_gyro = dT * (gx0 + gx) / (2 * GYRO_SENS);
	//delta_angle_y_gyro = dT * (gy0 + gy) / (2 * GYRO_SENS);
	
	// calculate complementary filtered x and y angles
	complementaryFilter(dT, delta_angle_x_gyro, delta_angle_y_gyro, angle_x, angle_y_accel, C_FILTER_T, angle_x_CF, angle_y_CF);

	// COMPLEMENTARY FILTER
	//--------------------------------------------------------------------------------------------------------------------------------------------
	*/
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// KALMAN FILTER
	
	// gyro rates
	static float rate_x_gyro;
	rate_x_gyro = gx / GYRO_SENS;
	
	// Kalman filtered x angle
	static float angle_x_KF;

	// calculate Kalman filtered x angle
	angle_x_KF = kalmanFilter_x.get_angle(dT, rate_x_gyro, angle_x);
	
	// KALMAN FILTER
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// NAVIGATION
	
	// velocity setpoint according to BB-1 remote data (determines the speed at which BB-1 is moving forward)
	static int16_t velocity_sp = 0;
	// delta velocity setpoint according to BB-1 remote data (determines the speed at which BB-1 is turning)
	static int16_t deltaVelocity_sp = 0;
	// receive update from BB-1 remote
	remote_update(velocity_sp, deltaVelocity_sp);
	
	// emergency braking status
	static boolean emergency_braking_front = false;
	static boolean emergency_braking_rear = false;
	// stay in emergency braking status until BB-1 has stopped in balance position (velocity < 200 mm/s and angle < 3°)
	if (emergency_braking_front || emergency_braking_rear) {
		if ((velocity < 200) && (angle_x_KF < 3)) {
			emergency_braking_front = false;
			emergency_braking_rear = false;
		}
	}
	
	// minimal allowed distance from detected objects in cm
	static const uint8_t min_distance = 70;
	// check min_distance and initiate emergency braking, if necessary in order to avoid a collision
	if (emergency_braking_front || ((front_distance < min_distance) && (velocity_sp > -200))) {
		emergency_braking_front = true;
		if (velocity > 0.6 * VELOCITY_LIMIT) {
			velocity_sp = -0.6 * VELOCITY_LIMIT;
		}
		else {
			velocity_sp = 0;
		}
	}
	if (emergency_braking_rear || ((rear_distance < min_distance) && (velocity_sp < 200))) {
		emergency_braking_rear = true;
		if (velocity < -0.6 * VELOCITY_LIMIT) {
			velocity_sp = 0.6 * VELOCITY_LIMIT;
		}
		else {
			velocity_sp = 0;
		}
	}
	
	// EMA filtered velocity setpoint
	static float velocity_sp_filtered = velocity_sp;
	// filter velocity setpoint
	velocity_sp_filtered = ema_filter(velocity_sp, velocity_sp_filtered, EMA_ALPHA_VELOCITY_SP);

	// EMA filtered delta velocity setpoint
	static float deltaVelocity_sp_filtered = deltaVelocity_sp;
	// filter delta velocity setpoint
	deltaVelocity_sp_filtered = ema_filter(deltaVelocity_sp, deltaVelocity_sp_filtered, EMA_ALPHA_DELTAVELOCITY_SP);
	
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(kalmanFilter_x.get_rate()); DEBUG_PRINT("\t"); DEBUG_PRINT(rate_x_gyro); DEBUG_PRINT("\t"); DEBUG_PRINTLN(kalmanFilter_x.get_rateBias());
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(kalmanFilter_x.get_rate()); DEBUG_PRINT("\t"); DEBUG_PRINT(rate_x_gyro); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_x_accel);
  
	static boolean failsafe = false;
	// enter failsafe if angle is too high
	if (abs(angle_x_KF) > ANGLE_MAX) {
		if (!failsafe) {
			// turn motors off
			md.setVelocities(0, 0);
			
			// reset PID controller
			pid_velocity_y.reset();
			pid_angle_x.reset();
			pid_deltaVelocity_y.reset();
			
			// set failsafe status
			failsafe = true;
			
			refresh = true;
		}
		
		// print display with PID values
		printDisplay(refresh, 1, velocity, angle_x_KF);
		
		return;
	}
	else {
		if (failsafe) {
			// reset failsafe status
			failsafe = false;
			
			refresh = true;
			
			// print display with sensor values
			printDisplay(refresh, 3, velocity, angle_x_KF);
			
			return;
		}
		
		// print display with sensor values
		printDisplay(first, 3, velocity, angle_x_KF);
	}

	
	// start balancing only if angle (angle_x_KF) is roughly in balance position (angle_x_KF < INIT_ANGLE) for a period of time (INIT_TIME)
	static boolean init = false;
	static int32_t init_timer = 0;
	static const int32_t INIT_TIME = 0;
	static const double INIT_ANGLE = 1;
	if (first || init) {
		if (abs(angle_x_KF) < INIT_ANGLE) {
			init_timer += dt;
		}
		else {
			init_timer = 0;
		}
		
		if (init_timer <= INIT_TIME) {
			init = true;
			return;
		}
		else {
			init = false;
			init_timer = 0;
		}
  }
	
	// NAVIGATION
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// CASCADED PID CONTROL
	
	// angle setpoint
	static float angle_x_sp;
	
	// control variables
	static float mv_M;
	static float mv_deltaM;
	
	// calculate angle setpoint
	angle_x_sp = pid_velocity_y.get_mv(velocity_sp_filtered, velocity_filtered, dT);
	
	// calculate control variable
	mv_M = pid_angle_x.get_mv(angle_x_sp, angle_x_KF, dT);
	mv_deltaM = pid_deltaVelocity_y.get_mv(deltaVelocity_sp_filtered, deltaVelocity_filtered, dT);
	
	// CASCADED PID CONTROL
	//--------------------------------------------------------------------------------------------------------------------------------------------



	/*// generate synthetic motor control inputs to test PIDs and filters - run it only without tires on!
 	static uint16_t t;
	t = millis();
	static uint16_t t0 = t;
	
	static float test_M = 0;
	static float test_deltaM = 0;
	
	static float amplitude = 0.7;
	static float amplitude_delta = 0.35;
	
	static float min_frequency = 0.05;  // 0.1
	static float max_frequency = 1;     // 2.0
	static float min_frequency_delta = 0.125;
	static float max_frequency_delta = 2.5;
	static float offset_delta = (2 * PI) * 0;
	
	static float frequency;
	static float frequency_delta;
	
	frequency = min_frequency + (max_frequency - min_frequency) * ((t - t0) * 0.0001);
	frequency_delta = min_frequency_delta + (max_frequency_delta - min_frequency_delta) * ((t - t0) * 0.0001);
	
	if ((t - t0) > 3000) {
		test_M = amplitude * 255 * sin(frequency * 0.006283 * (t - t0));
		test_deltaM = amplitude_delta * 255 * sin(frequency_delta * 0.006283 * (t - t0) + offset_delta);
		if (((t - t0) > 13000)) {
			test_M = 0;
			test_deltaM = 0;
			t0 = t;
		}
	}*/
	
	
	
	// set motor velocities
	md.setVelocities(constrain(round(mv_M - mv_deltaM), -255, 255), constrain(round(mv_M + mv_deltaM), -255, 255));
	//md.setVelocities(constrain(round(test_M - mv_deltaM), -255, 255), constrain(round(test_M + test_deltaM + mv_deltaM), -255, 255));
	//md.setVelocities(constrain(round(test_M), -255, 255), constrain(round(test_M), -255, 255));
	//md.setVelocities(test_M, test_M);
	//md.setVelocities(0, 0);
	
	// check if motor shield reports error
	if (md.getFault())
	{
		DEBUG_PRINT("Error: Motor Shield fault!");
		while(1);
	}
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// SERIAL DEBUG
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
	//DEBUG_PRINT(gx); DEBUG_PRINT("\t"); DEBUG_PRINT(gy); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz);
	
	//DEBUG_PRINT(enc_count_M1); DEBUG_PRINT("\t"); DEBUG_PRINTLN(enc_count_M2);
	//DEBUG_PRINT(front_distance); DEBUG_PRINT("\t"); DEBUG_PRINTLN(rear_distance);
	
	//DEBUG_PRINT(test_M); DEBUG_PRINT("\t"); DEBUG_PRINT(test_deltaM);  DEBUG_PRINT("\t"); DEBUG_PRINTLN(test_M + test_deltaM);
	//DEBUG_PRINT(velocity_M1_filtered); DEBUG_PRINT("\t"); DEBUG_PRINTLN(velocity_M2_filtered);
	
	//DEBUG_PRINT(velocity); DEBUG_PRINT("\t"); DEBUG_PRINTLN(velocity_filtered);
	
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(kalmanFilter_x.get_rate()); DEBUG_PRINT("\t"); DEBUG_PRINT(rate_x_gyro); DEBUG_PRINT("\t"); DEBUG_PRINTLN(kalmanFilter_x.get_rateBias());
	
	//DEBUG_PRINTLN(dt);
	
	//DEBUG_PRINT(velocity_sp_filtered); DEBUG_PRINT("\t"); DEBUG_PRINTLN(velocity_filtered);
	//DEBUG_PRINT(velocity_M1); DEBUG_PRINT("\t"); DEBUG_PRINT(velocity_M2);  DEBUG_PRINT("\t"); DEBUG_PRINTLN(velocity_filtered);
	
	//DEBUG_PRINTLN(angle_x_sp);
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_x_gyro);
	
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(kalmanFilter_x.get_rate()); DEBUG_PRINT("\t"); DEBUG_PRINTLN(kalmanFilter_x.get_rateBias());
	//DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x); DEBUG_PRINTLN("\t"); DEBUG_PRINTLN(angle_x_gyro);
	
	//DEBUG_PRINTLN(line_1);
	//DEBUG_PRINTLN(line_2);
	//DEBUG_PRINTLN(line_3);
	
	//DEBUG_PRINT(dt); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mpu_update_time);
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x_CF); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x_gyro); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_x);
	
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_x_sp); DEBUG_PRINT("\t"); DEBUG_PRINT(velocity_filtered/100); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mv_M/10);
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_x_sp);
	//DEBUG_PRINT(deltaVelocity); DEBUG_PRINT("\t"); DEBUG_PRINTLN(deltaVelocity_sp_filtered);
	
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(kalmanFilter_x.get_rate()); DEBUG_PRINT("\t"); DEBUG_PRINTLN(kalmanFilter_x.get_rateBias());
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINTLN(cv_M1_pwm);
	//DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(mv_M); DEBUG_PRINT("\t");
	//DEBUG_PRINT(md.getM1Current()); DEBUG_PRINT("\t"); DEBUG_PRINTLN(md.getM2Current());
	
	// Send data to "Processing" for visualization
	//sendSerial(angle_x_gyro, angle_y_gyro, angle_z_gyro, angle_x_accel, angle_y_accel, angle_x_CF, angle_y_CF, angle_x_KF, angle_y_KF);
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// SERIAL DEBUG
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	// update last gyro values
	gx0 = gx;
	gy0 = gy;
	gz0 = gz;
}

// update from BB-1 remote
void remote_update(int16_t& velocity_sp, int16_t& deltaVelocity_sp) {
	static boolean remoteConnection = false;	// remote connection status
	static boolean requestRemoteData = true;	// specifies if new data can be requested from remote
	static int32_t reconnectTime = 0;		// time since last try to reconnect in microseconds
	static char receivedChars[MAX_CHARS];	// received chars
	
	// check if remote appears to be connected
	if (remoteConnection) {
		if (getData(VELOCITY, receivedChars, remoteConnection, requestRemoteData)) {
			velocity_sp_update(receivedChars, velocity_sp, deltaVelocity_sp);
		}
	}
	else {
		// set velocity setpoints to zero, so BB-1 does not drive away when the remote connection is lost
		velocity_sp = 0;
		deltaVelocity_sp = 0;
		
		// try to connect at start or try to reconnect in a time interval specified by REMOTE_TIMEOUT
		if (first || ((reconnectTime += dt) > REMOTE_TIMEOUT)) {
			Serial1.print(HELLO_BB1_REMOTE);
			reconnectTime = 0;
		}
		
		if (Serial1.available() > 0) {
			if (Serial1.read() == HELLO_BB1) {
				remoteConnection = true;
				requestRemoteData = true;
				reconnectTime = 0;
			}
		}
	}
}

// get data from BB-1 remote
boolean getData(char order, char* receivedChars, boolean& remoteConnection, boolean& requestRemoteData) {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	static char rc;
	static int32_t answerTime = 0;	// time since last data request in microseconds
	
	if (requestRemoteData) {
		// request data from BB-1 remote
		Serial1.print(order);
		
		requestRemoteData = false;
		answerTime = 0;
	}
	else {
		// check for communication timeout
		if ((answerTime += dt) < REMOTE_TIMEOUT) {
			while (Serial1.available()) {
				answerTime = 0;
				
				rc = Serial1.read();
				
				if (recvInProgress) {
					if (rc != '>') {
						receivedChars[ndx] = rc;
						ndx++;
						if (ndx >= MAX_CHARS) {
							ndx = MAX_CHARS - 1;
						}
					}
					else {
						receivedChars[ndx] = '\0';	// terminate the string
						ndx = 0;
						recvInProgress = false;
						requestRemoteData = true;
						
						// check if the received data matches the ordered data
						if (receivedChars[0] == order) {
							return true;
						}
					}
				}
				else if (rc == '<') {
					recvInProgress = true;
				}
			}
		}
		else {
			remoteConnection = false;
			recvInProgress = false;
			ndx = 0;
		}
	}
	return false;
}

// update velocity setpoints
void velocity_sp_update(char* receivedChars, int16_t& velocity_sp, int16_t& deltaVelocity_sp) {
	char *strtokIndx;
	
	// joystick data
	static int8_t x;
	static int8_t y;
	
	// get dataType
	strtokIndx = strtok(receivedChars,",");
	// check if the received data type matches the requested data type
	if (VELOCITY == strtokIndx[0]) {
		// get x data
		strtokIndx = strtok(NULL, ",");
		x = atoi(strtokIndx);
		// get y data
		strtokIndx = strtok(NULL, ",");
		y = atoi(strtokIndx);

		// update velocity setpoint
		velocity_sp = map(x, -127, 127, -VELOCITY_LIMIT, VELOCITY_LIMIT);
		
		// update delta velocity setpoint
		deltaVelocity_sp = map(y, -127, 127, DELTA_VELOCITY_LIMIT, -DELTA_VELOCITY_LIMIT);
	}
}

// update accel, gyro and encoder values and measure update time
void sensor_update() {
	// variables to measure MPU update time
	static uint32_t t0 = 0;
	static uint32_t t = 0;
	
	if (first)  {
		while (!mpuInterrupt) {
			// wait for the next interrupt
		}
		
		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		t0 = micros();

		// read raw accel/gyro values
		mpu.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);
		
		// request message from slave device
		Wire.requestFrom(SLAVE_ADDRESS, 4);
		// read encoder counts
		enc_count_M1 = Wire.read();
		enc_count_M2 = Wire.read();
		// read distances in cm
		front_distance = Wire.read();
		rear_distance = Wire.read();
	}
	
	while (!mpuInterrupt) {
		// wait for the next interrupt
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	t = micros();

	// read raw accel/gyro values
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	// request message from slave device
	Wire.requestFrom(SLAVE_ADDRESS, 4);
	// read encoder counts
	enc_count_M1 = Wire.read();
	enc_count_M2 = Wire.read();
	// read distances in cm
	front_distance = Wire.read();
	rear_distance = Wire.read();

	dt = (t - t0);				// in us
	dT = float(dt) * 0.000001;	// in s
	
	// update last MPU update time measurement
	t0 = t;
}

// calculate velocities
void calc_velocities(float& velocity_M1, float& velocity_M2) {
	static const float TEMP = (float) TIRE_DIAMETER * PI / ENCODER_RESOLUTION;
	static float temp;
	
	temp = TEMP / dT;
	
	velocity_M1 = enc_count_M1 * temp;
	velocity_M2 = enc_count_M2 * temp;
}

// calculate accel x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel) {
	// calculate accel x and y angles
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	//angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calculate gyro x, y and z angles in degrees (not necessary for filters)
void calc_gyroAngles(float& angle_x_gyro, float& angle_y_gyro, float& angle_z_gyro) {
	// calculated gyro angles (LSB / 2000000) - initialized with starting angle
	static int64_t angle_x_gyro_scaled = angle_x_gyro * 2000000 * GYRO_SENS;
	//static int64_t angle_y_gyro_scaled = angle_y_gyro * 2000000 * GYRO_SENS;
	//static int64_t angle_z_gyro_scaled = angle_z_gyro * 2000000 * GYRO_SENS;
	
	// calculate gyro angles (LSB / 2000000)
	angle_x_gyro_scaled += dt * (gx0 + gx);
	//angle_y_gyro_scaled += dt * (gy0 + gy);
	//angle_z_gyro_scaled += dt * (gz0 + gz);

	// make sure gyro angles do not grow endlessly
	const static int64_t TEMP = (360 * 2000000 * GYRO_SENS);
	angle_x_gyro_scaled = angle_x_gyro_scaled % TEMP;
	//angle_y_gyro_scaled = angle_y_gyro_scaled % TEMP;
	//angle_z_gyro_scaled = angle_z_gyro_scaled % TEMP;

	// calculate gyro x, y and z angles
	angle_x_gyro = angle_x_gyro_scaled / (2000000 * GYRO_SENS);
	//angle_y_gyro = angle_y_gyro_scaled / (2000000 * GYRO_SENS);
	//angle_z_gyro = angle_z_gyro_scaled / (2000000 * GYRO_SENS);
}

// calculate x angle of BB1 in degrees, which, compared to accel x angle, is corrected by the error caused by BB-1 acceleration
void calc_angleX(float angle_x_accel, float velocity, float& angle_x) {
	static float velocity_old = velocity;

	// calculate x angle of BB1 in degrees
	angle_x = angle_x_accel + RAD2DEG * atan2((velocity - velocity_old) / dT, G);
	
	// update old velocity
	velocity_old = velocity;
}

// print display (mode 1: PID; mode 2: average velocity, angle, dt; mode 3: average velocity, angle, minimal distance)
void printDisplay(boolean refresh, int8_t mode, float velocity, float angle_x_KF) {
	static const int32_t LCD_REFRESH_TIME = 500000;
	static int32_t LCD_time = 0;
	
	static int16_t LCD_temp1, LCD_temp2;
	
	// print menu
	if (refresh) {
		lcd.clear();
		switch (mode) {
			case 1:
				lcd.setCursor (0, 0);
				lcd.print("        BB-1        ");
				lcd.setCursor (0, 1);
				lcd.print("P:");
				lcd.setCursor (5, 1);
				lcd.print(".");
				lcd.setCursor (0, 2);
				lcd.print("I:");
				lcd.setCursor (5, 2);
				lcd.print(".");
				lcd.setCursor (0, 3);
				lcd.print("D:");
				lcd.setCursor (5, 3);
				lcd.print(".");
				break;
				
			case 2:
				lcd.setCursor (0, 0);
				lcd.print("        BB-1        ");
				lcd.setCursor (0, 1);
				lcd.print("v:");
				lcd.setCursor (7, 1);
				lcd.print(".");
				lcd.setCursor (17, 1);
				lcd.print("m/s");
				lcd.setCursor (0, 2);
				lcd.print("a:");
				lcd.setCursor (7, 2);
				lcd.print(".");
				lcd.setCursor (17, 2);
				lcd.print((char)223);
				lcd.setCursor (0, 3);
				lcd.print("dt:");
				lcd.setCursor (7, 3);
				lcd.print(".");
				lcd.setCursor (17, 3);
				lcd.print("ms");
				break;
				
			case 3:
				lcd.setCursor (0, 0);
				lcd.print("        BB-1        ");
				lcd.setCursor (0, 1);
				lcd.print("v:");
				lcd.setCursor (7, 1);
				lcd.print(".");
				lcd.setCursor (17, 1);
				lcd.print("m/s");
				lcd.setCursor (0, 2);
				lcd.print("a:");
				lcd.setCursor (7, 2);
				lcd.print(".");
				lcd.setCursor (17, 2);
				lcd.print((char)223);
				lcd.setCursor (0, 3);
				lcd.print("d:");
				lcd.setCursor (17, 3);
				lcd.print("cm");
				break;
				
			default:
				break;
		}
		
		// reset interrupt status
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();
	}
	
	// print values
	if (LCD_time >= LCD_REFRESH_TIME) {
		LCD_time = 0;
		switch (mode) {
			case 1:
				lcd.setCursor(3, 1);
				LCD_temp2 = round(P_angle * 10);
				LCD_temp1 = LCD_temp2 / 10;
				LCD_temp2 = LCD_temp2 % 10;
				if (LCD_temp1 < 10) {
					lcd.print(" ");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
			
				lcd.setCursor (3, 2);
				LCD_temp2 = round(I_angle * 10);
				LCD_temp1 = LCD_temp2 / 10;
				LCD_temp2 = LCD_temp2 % 10;
				if (LCD_temp1 < 10) {
					lcd.print(" ");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
			
				lcd.setCursor (4, 3);
				LCD_temp2 = round(D_angle * 100);
				LCD_temp1 = LCD_temp2 / 100;
				LCD_temp2 = LCD_temp2 % 100;
				lcd.print(LCD_temp1); lcd.moveCursorRight();
				if (LCD_temp2 < 10) {
					lcd.print("0");
				}
				lcd.print(LCD_temp2);
				break;
				
			case 2:
				lcd.setCursor (5, 1);
				LCD_temp2 = round(velocity * 0.01);
				LCD_temp1 = abs(LCD_temp2 / 10);
				LCD_temp2 = abs(LCD_temp2 % 10);
				if (velocity >= 0) {
					lcd.print(" ");
				}
				else {
					lcd.print("-");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
			
				lcd.setCursor (3, 2);
				LCD_temp2 = round(angle_x_KF * 10);
				LCD_temp1 = abs(LCD_temp2 / 10);
				LCD_temp2 = abs(LCD_temp2 % 10);
				if (LCD_temp1 < 100) {
					lcd.print(" ");
					if (LCD_temp1 < 10) {
						lcd.print(" ");
					}
				}
				if (angle_x_KF >= 0) {
					lcd.print(" ");
				}
				else {
					lcd.print("-");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
			
				lcd.setCursor (5, 3);
				LCD_temp2 = round(dt * 0.01);
				LCD_temp1 = abs(LCD_temp2 / 10);
				LCD_temp2 = abs(LCD_temp2 % 10);
				if (LCD_temp1 < 10) {
					lcd.print(" ");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
				break;

			case 3:
				lcd.setCursor (5, 1);
				LCD_temp2 = round(velocity * 0.01);
				LCD_temp1 = abs(LCD_temp2 / 10);
				LCD_temp2 = abs(LCD_temp2 % 10);
				if (velocity >= 0) {
					lcd.print(" ");
				}
				else {
					lcd.print("-");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
			
				lcd.setCursor (3, 2);
				LCD_temp2 = round(angle_x_KF * 10);
				LCD_temp1 = abs(LCD_temp2 / 10);
				LCD_temp2 = abs(LCD_temp2 % 10);
				if (LCD_temp1 < 100) {
					lcd.print(" ");
					if (LCD_temp1 < 10) {
						lcd.print(" ");
					}
				}
				if (angle_x_KF >= 0) {
					lcd.print(" ");
				}
				else {
					lcd.print("-");
				}
				lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
				
				// lowest distance
				static uint8_t lowest_distance;
				if (front_distance < rear_distance) {
					lowest_distance = front_distance;
				}
				else {
					lowest_distance = rear_distance;
				}
				
				lcd.setCursor (6, 3);
				if (lowest_distance < 10) {
					lcd.print("  ");
				}
				else if (lowest_distance < 100) {
					lcd.print(" ");
				}
				lcd.print(lowest_distance);
				break;
				
			default:
				break;
		}
	}
	else {
		LCD_time += dt;
	}
}

// calibrate MPU
void calibMotion6(int16_t num, int16_t accuracy_accel, int16_t accuracy_gyro) {
	uint8_t step_counter = 0;

	int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	int16_t step_ax = 0, step_ay = 0, step_az = 0, step_gx = 0, step_gy = 0, step_gz = 0;
	int16_t offset_ax = 0, offset_ay = 0, offset_az = 0, offset_gx = 0, offset_gy = 0, offset_gz = 0;

	while (1) {
		step_counter++;
		
		mpu.setXAccelOffset(offset_ax);
		mpu.setYAccelOffset(offset_ay);
		mpu.setZAccelOffset(offset_az);

		mpu.setXGyroOffset(offset_gx);
		mpu.setYGyroOffset(offset_gy);
		mpu.setZGyroOffset(offset_gz);

		meanMotion6(num, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);

		if ((abs(mean_ax) < accuracy_accel) &&
		(abs(mean_ay) < accuracy_accel) &&
		((abs(mean_az - G_LSB) < accuracy_accel) && (mean_az >= -G_LSB * pow(2, ACCEL_RANGE + 1) - 1)) &&
		(abs(mean_gx) < accuracy_gyro) &&
		(abs(mean_gy) < accuracy_gyro) &&
		(abs(mean_gz) < accuracy_gyro)) {
			break;
		}

		step_ax = mean_ax * (pow(2, ACCEL_RANGE) / 9);
		step_ay = mean_ay * (pow(2, ACCEL_RANGE) / 9);
		step_az = mean_az * (pow(2, ACCEL_RANGE) / 9) - G_LSB  * (pow(2, ACCEL_RANGE) / 9);

		step_gx = mean_gx * (pow(2, GYRO_RANGE) / 4);
		step_gy = mean_gy * (pow(2, GYRO_RANGE) / 4);
		step_gz = mean_gz * (pow(2, GYRO_RANGE) / 4);

		offset_ax -= step_ax;
		offset_ay -= step_ay;
		offset_az -= step_az;

		offset_gx -= step_gx;
		offset_gy -= step_gy;
		offset_gz -= step_gz;
	}

	DEBUG_PRINT("Step count:\t");
	DEBUG_PRINT(step_counter);
	DEBUG_PRINT("\n");

	DEBUG_PRINTLN(F("Updated internal sensor offsets:"));
	DEBUG_PRINT("a/g:\t");
	DEBUG_PRINT(mpu.getXAccelOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mpu.getYAccelOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mpu.getZAccelOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mpu.getXGyroOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mpu.getYGyroOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mpu.getZGyroOffset());
	DEBUG_PRINT("\t");
	DEBUG_PRINT("\n");

	DEBUG_PRINTLN(F("Mean measurement error:"));
	DEBUG_PRINT("a/g:\t");
	DEBUG_PRINT(mean_ax);
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mean_ay);
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mean_az - G_LSB);
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mean_gx);
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mean_gy);
	DEBUG_PRINT("\t");
	DEBUG_PRINT(mean_gz);
	DEBUG_PRINT("\n");
}

// calculate mean MPU measurements
void meanMotion6(int16_t num, int16_t& mean_ax, int16_t& mean_ay, int16_t& mean_az, int16_t& mean_gx, int16_t& mean_gy, int16_t& mean_gz)  {
	int32_t sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;

	for (int16_t i = 0; i < num; i++)  {
		while (!mpuInterrupt) {
			// wait for the next interrupt
		}

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		sum_ax += ax;
		sum_ay += ay;
		sum_az += az;
		sum_gx += gx;
		sum_gy += gy;
		sum_gz += gz;
	}

	mean_ax = sum_ax / num;
	mean_ay = sum_ay / num;
	mean_az = sum_az / num;
	mean_gx = sum_gx / num;
	mean_gy = sum_gy / num;
	mean_gz = sum_gz / num;
}


// send serial data to Processing
void sendSerial(float angle_x_gyro, float angle_y_gyro, float angle_z_gyro, float angle_x_accel, float angle_y_accel, float angle_x_CF, float angle_y_CF, float angle_x_KF, float angle_y_KF) {
	// send dt, accelerometer angles, gyro angles and filtered angles
	Serial.print(F("DEL:"));
	Serial.print(dt, DEC);
	//Serial.print(F("#GYR:"));
	//Serial.print(angle_x_gyro, 2); Serial.print(F(",")); Serial.print(angle_y_gyro, 2);	Serial.print(F(",")); Serial.print(angle_z_gyro, 2);
	//Serial.print(F("#ACC:"));
	//Serial.print(angle_x_accel, 2);	Serial.print(F(",")); Serial.print(angle_y_accel, 2);
	//Serial.print(F("#CFI:"));
	//Serial.print(angle_x_CF, 2); Serial.print(F(",")); Serial.print(angle_y_CF, 2);
	Serial.print(F("#KFI:"));
	Serial.print(angle_x_KF, 2); Serial.print(F(",")); Serial.print(angle_y_KF, 2);
	Serial.println(F(""));
}
