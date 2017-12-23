/*
* BB-1.ino
*
* Created: 05.11.2016
* Author: Lukas
*
* Based on I2C device class (I2Cdev) and MPU6050 class by Jeff Rowberg (Arduino)
*
*/

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "DualMC33926MotorShield.h"
#include "LiquidCrystal_I2C.h"

// #include "complementary_filter.h"
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
// MPU6050 mpu(0x69); // <-- use for AD0 high

// set the LCD address to 0x3F for a 20 chars 4 line display and
// set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// address of the Arduino slave reading encoder values
#define SLAVE_ADDRESS 0x08

boolean first = true;

const float RAD2DEG = 4068 / 71;

// acceleration of gravity
const float G = 9.81;

// configuration for digital low pass filter
const char DLPFMode = MPU6050_DLPF_BW_98;

// sample rate = gyroscope output rate / (1 + SampleRateDivider)
// gyroscope output rate is 8kHz for DLPFMode MPU6050_DLPF_BW_256 else 1 kHz
const char SampleRateDivider = 6;

// targeted MPU update time in microseconds
int32_t mpu_update_time;

// measured MPU update time in microseconds
int32_t dt = 0;
// measured MPU update time in seconds
float dT = 0;

// variables to measure MPU update time
uint32_t t0;
uint32_t t;

// range of accelerometer and gyro
const int16_t ACCEL_RANGE = MPU6050_ACCEL_FS_4;
const int16_t GYRO_RANGE = MPU6050_GYRO_FS_1000;

// acceleration of gravity in LSB
const int16_t G_LSB = pow(2, 16) / (4 * pow(2, ACCEL_RANGE));

// MPU sensitivity
const float ACCEL_SENS = pow(2, 16) / (4 * pow(2, ACCEL_RANGE)) / G;
const float GYRO_SENS = pow(2, 16) / (500 * pow(2, GYRO_RANGE));

// encoder resolution in counts per revolution
const int16_t ENCODER_RESOLUTION = 816;

// wheel diameter in mm
const int16_t WHEEL_DIAMETER = 80;

// MPU raw measurements
int16_t ax, ay, az;
int16_t gx, gy, gz;

// last MPU raw measurements
int16_t ax0, ay0, az0;
int16_t gx0, gy0, gz0;

// encoder raw measurements
int8_t enc_count_M1, enc_count_M2;

// Kalman filter class
KalmanFilter kalmanFilter_x(1, 0.1, 0.05, 0.1, 0.001, 0);	// previous parameters: (1, 0.01, 0.03, 0.1, 0.001, 0), qp_rate to r_gyro ratio is important
// KalmanFilter kalmanFilter_y(1, 0.1, 0.05, 0.1, 0.001, 0);	// previous parameters: (1, 0.01, 0.03, 0.1, 0.001, 0), qp_rate to r_gyro ratio is important

// turn on PID tuning with potentiometers
static boolean tunePID = true;

// PID values for angle controller
float P_angle = 3;
float I_angle = 0;
float D_angle = 0;

// PID values for velocity controller
float P_velovcity = 0;
float I_velovcity = 0;
float D_velovcity = 0;

// PID controller classes for angle (mpu) and velocity (encoder)
PID_controller pid_angle_x_1(P_angle, I_angle, D_angle, 0, 5, 255);
PID_controller pid_angle_x_2(P_angle, I_angle, D_angle, 0, 0, 255);
PID_controller pid_velocity_y_1(P_velovcity, I_velovcity, D_velovcity, 0, 0, 45);
PID_controller pid_velocity_y_2(P_velovcity, I_velovcity, D_velovcity, 0, 0, 45);

// motor controller class
DualMC33926MotorShield md(11, 9, A0, 8, 10, A1, 4, 12);	// remap M1DIR from pin 7 to pin 11

// uncomment "PERFORM_MPU_CALIBRATION" if you want to calibrate the MPU
// #define PERFORM_MPU_CALIBRATION

#define MPU_INTERRUPT_PIN 7

// potentiometer pins to control PID values
#define P_PIN A3
#define I_PIN A4
#define D_PIN A5

bool blinkState = false;

// MPU interrupt status byte
uint8_t mpuIntStatus;

// read sensor data and measure update time
void sensor_update();

// calculate velocities
void calc_velocities();

// calculate accel x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel);

// calculate gyro x, y and z angles in degrees (not necessary for filters)
void calc_gyroAngles(float& angle_x_gyro, float& angle_y_gyro, float& angle_z_gyro);

// print display
void printDisplay(int8_t mode, float velocity_M1, float velocity_M2, float angle_x_KF);

// calibrate MPU
void calibMotion6(const int16_t NUM, const int16_t ACCURACY_ACCEL, const int16_t ACCURACY_GYRO);

// calculate mean MPU measurements
void meanMotion6(const int16_t NUM, int16_t& mean_ax, int16_t& mean_ay, int16_t& mean_az, int16_t& mean_gx, int16_t& mean_gy, int16_t& mean_gz);

// send serial data to Processing
void sendSerial(int32_t dt, float angle_x_gyro, float angle_y_gyro, float angle_z_gyro, float angle_x_accel, float angle_y_accel,
float angle_x_CF, float angle_y_CF, float angle_x_KF, float angle_y_KF);


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dataReady() {
	mpuInterrupt = true;
}

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


void setup() {
	// calibration parameter
	const int16_t NUM = 100;
	const int16_t ACCURACY_ACCEL = 9;
	const int16_t ACCURACY_GYRO = 4;

	// configure Arduino interrupt pin for MPU
	pinMode(MPU_INTERRUPT_PIN, INPUT);
	digitalWrite(MPU_INTERRUPT_PIN, HIGH);

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

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Arduino
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize I2C devices
	DEBUG_PRINTLN(F("\nInitializing I2C devices..."));
	
	// initialize LCD (seems to negatively affect interrupts)
	lcd.begin(20, 4);
	
	// initialize bluetooth module
	
	// initialize MPU
	mpu.initialize();
	mpu.setDLPFMode(DLPFMode);
	mpu.setRate(SampleRateDivider);
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
	// MPU update time = 1 / MPU sample rate = (SampleRateDivider + 1) / gyroscope output rate
	// The gyroscope output rate depends on the Low Pass Filter used (DLPFMode)
	if (DLPFMode == MPU6050_DLPF_BW_256) {	// gyroscope output rate = 8 kHz
		mpu_update_time = (SampleRateDivider + 1) * 125;	// mpu_update_time in microseconds, 1000000 / 8000 = 125
	}
	else {	// gyroscope output rate = 1 kHz
		mpu_update_time = (SampleRateDivider + 1) * 1000;	// mpu_update_time in microseconds, 1000000 / 1000 = 1000
	}
	
	#ifdef PERFORM_MPU_CALIBRATION
	// calibrate MPU
	DEBUG_PRINTLN(F("\nCalibrating ..."));
	calibMotion6(NUM, ACCURACY_ACCEL, ACCURACY_GYRO);
	DEBUG_PRINTLN(F("Calibration completed.\n"));
	#else
	// use offsets from previous calibration
	mpu.setXAccelOffset(-506);
	mpu.setYAccelOffset(2524);
	mpu.setZAccelOffset(1047);

	mpu.setXGyroOffset(-10);
	mpu.setYGyroOffset(34);
	mpu.setZGyroOffset(52);
	#endif
}


void loop() {	
	// read sensor data and measure update time
	sensor_update();
	
	// velocities
	static float velocity_M1, velocity_M2;
	// calculate velocities
	calc_velocities(velocity_M1, velocity_M2);

	// accel angles
	static float angle_x_accel, angle_y_accel;
	// calculate accel x and y angles in degrees
	calc_accelAngles(angle_x_accel, angle_y_accel);
	
	// gyro angles
	// static float angle_x_gyro = 0, angle_y_gyro = 0, angle_z_gyro = 90;
	// calculate gyro x, y and z angles in degrees (not necessary for filters)
	// calc_gyroAngles(angle_x_gyro, angle_y_gyro, angle_z_gyro);
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// COMPLEMENTARY FILTER
	/*
	// complementary filter time constant
	static const float C_FILTER_T = 0.3;
	
	// delta gyro angles
	static float delta_angle_x_gyro, delta_angle_y_gyro;
	
	// complementary filtered angles
	static float angle_x_CF = 0, angle_y_CF = 0;
	
	// delta gyro angles
	delta_angle_x_gyro = dT * (gx0 + gx) / 2 / GYRO_SENS;
	delta_angle_y_gyro = dT * (gy0 + gy) / 2 / GYRO_SENS;
	
	// calculate complementary filtered x and y angles
	complementaryFilter(angle_x_CF, angle_y_CF, dT, delta_angle_x_gyro, delta_angle_y_gyro, angle_x_accel, angle_y_accel, C_FILTER_T);
	*/
	// COMPLEMENTARY FILTER
	//--------------------------------------------------------------------------------------------------------------------------------------------

	//--------------------------------------------------------------------------------------------------------------------------------------------
	// KALMAN FILTER
	
	// gyro rates
	static float rate_x_gyro; static float rate_y_gyro; static float rate_z_gyro;
	
	rate_x_gyro = gx / GYRO_SENS;
	// rate_y_gyro = gy / GYRO_SENS;
	// rate_z_gyro = gz / GYRO_SENS;
	
	// Kalman filtered angles
	static float angle_x_KF, angle_y_KF;

	// calculate Kalman filtered x and y angles
	angle_x_KF = kalmanFilter_x.get_angle(dT, rate_x_gyro, angle_x_accel);
	// angle_y_KF = kalmanFilter_y.get_angle(dT, rate_y_gyro, angle_y_accel);
	
	// KALMAN FILTER
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// MADGWICK FILTER
	// ... TODO ...
	// MADGWICK FILTER
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	// update old values
	t0 = t;
	
	gx0 = gx;
	gy0 = gy;
	gz0 = gz;
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// NAVIGATION
	
	if (tunePID)
	{
		P_angle = (float) constrain(analogRead(P_PIN) - 10, 0, 999) / 10;
		I_angle = (float) constrain(analogRead(I_PIN) - 10, 0, 999) / 10;
		D_angle = (float) constrain(analogRead(D_PIN) - 10, 0, 999) / 100;
		
		pid_angle_x_1.set_K_p(P_angle);
		pid_angle_x_1.set_K_i(I_angle);
		pid_angle_x_1.set_K_d(D_angle);
		pid_angle_x_2.set_K_p(P_angle);
		pid_angle_x_2.set_K_i(I_angle);
		pid_angle_x_2.set_K_d(D_angle);
	}	
	
	// failsafe if angle is too high
	if (abs(angle_x_KF) > 40) {
		// turn motors off
		md.setVelocities(0, 0);
		
		// reset PID controller
		pid_velocity_y_1.reset();
		pid_velocity_y_2.reset();
		pid_angle_x_1.reset();
		pid_angle_x_2.reset();
		
		// print display with PID values
		printDisplay(2, velocity_M1, velocity_M2, angle_x_KF);
		
		return;
	}
	else {
		// print display with sensor values
		printDisplay(1, velocity_M1, velocity_M2, angle_x_KF);
	}
	
	// NAVIGATION
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// CASCADED PID CONTROL
	
	// velocity
	static float velocity_y_1 = 0;
	static float velocity_y_2 = 0;
	// velocity setpoint
	static float velocity_y_sp = 0;
	
	// angle setpoint
	static float angle_x_sp_1;
	static float angle_x_sp_2;
	
	// control variables
	static float mv_M1;
	static float mv_M2;
	
	// calculate angle setpoint
	angle_x_sp_1 = pid_velocity_y_1.get_mv(velocity_y_sp, velocity_y_1, dT);
	angle_x_sp_2 = pid_velocity_y_2.get_mv(velocity_y_sp, velocity_y_2, dT);
	
	// calculate control variables
	mv_M1 = pid_angle_x_1.get_mv(angle_x_sp_1, angle_x_KF, dT);
	mv_M2 = pid_angle_x_2.get_mv(angle_x_sp_2, angle_x_KF, dT);
	
	// CASCADED PID CONTROL
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	// set motor velocities
	md.setVelocities(round(mv_M1), round(mv_M2));
	
	// check if motor shield reports error
	if (md.getFault())
	{
		DEBUG_PRINT("Error: Motor Shield fault!");
		while(1);
	}
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// SERIAL DEBUG
	//--------------------------------------------------------------------------------------------------------------------------------------------
	
	// DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
	// DEBUG_PRINT(gx); DEBUG_PRINT("\t"); DEBUG_PRINT(gy); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz);
	
	// DEBUG_PRINT(enc_count_M1); DEBUG_PRINT("\t"); DEBUG_PRINTLN(enc_count_M2);
	
	// DEBUG_PRINTLN(dt);
	
	// DEBUG_PRINTLN(line_1);
	// DEBUG_PRINTLN(line_2);
	// DEBUG_PRINTLN(line_3);
	
	// DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(velocity_M1); DEBUG_PRINT("\t"); DEBUG_PRINTLN(velocity_M2);
	
	// DEBUG_PRINT(dt); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mpu_update_time);
	DEBUG_PRINTLN(angle_x_KF);
	
	// DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINTLN(cv_M1_pwm);
	// DEBUG_PRINT(angle_x_KF); DEBUG_PRINT("\t"); DEBUG_PRINT(mv_M1); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mv_M2);
	// DEBUG_PRINT(md.getM1Current()); DEBUG_PRINT("\t"); DEBUG_PRINTLN(md.getM2Current());
	
	// Send data to "Processing" for visualization
	// sendSerial(dt, angle_x_gyro, angle_y_gyro, angle_z_gyro, angle_x_accel, angle_y_accel, angle_x_CF, angle_y_CF, angle_x_KF, angle_y_KF);
	
	//--------------------------------------------------------------------------------------------------------------------------------------------
	// SERIAL DEBUG
	//--------------------------------------------------------------------------------------------------------------------------------------------
}

// read sensor data and measure update time
void sensor_update() {	
	// print error when interrupts happen too early or too late
	if (abs((dt * 100) / mpu_update_time - 100) > 3) {
		if (!first) {
			if (((dt * 100) / mpu_update_time - 100) > 3) {
				//DEBUG_PRINTLN("Error: MPU interrupt too early!");
			}
			else {
				//DEBUG_PRINTLN("Warning: MPU interrupt too late!");
			}
			
			//DEBUG_PRINT("Expected: "); DEBUG_PRINT(mpu_update_time); DEBUG_PRINT(" us"); DEBUG_PRINT("\t"); DEBUG_PRINT("Measured: "); DEBUG_PRINT(dt); DEBUG_PRINTLN(" us");
			
			//while(1);
			
			// reset interrupt status
			// mpuInterrupt = false;
			// mpuIntStatus = mpu.getIntStatus();
			// first = true;
		}
	}
	
	while (!mpuInterrupt) {
		// wait for the next interrupt
	}
	
	if (first)  {
		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		t0 = micros();

		// read raw accel/gyro values
		mpu.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);
		
		// request two encoder counts (2 * 1 byte) from slave device
		Wire.requestFrom(SLAVE_ADDRESS, 2);
		// read encoder counts
		enc_count_M1 = Wire.read();
		enc_count_M2 = Wire.read();

		first = false;

		while (!mpuInterrupt) {
			// wait for the next interrupt
		}
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	t = micros();

	// read raw accel/gyro values
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	// request two encoder counts (2 * 1 byte) from slave device
	Wire.requestFrom(SLAVE_ADDRESS, 2);
	// read encoder counts
	enc_count_M1 = Wire.read();
	enc_count_M2 = Wire.read();

	dt = (t - t0);	// in us
	dT = float(dt) / 1000000;	// in s
}

// calculate velocities
void calc_velocities(float& velocity_M1, float& velocity_M2)
{
	velocity_M1 = (float)enc_count_M1 / ENCODER_RESOLUTION * WHEEL_DIAMETER * PI / dT;
	velocity_M2 = (float)enc_count_M2 / ENCODER_RESOLUTION * WHEEL_DIAMETER * PI / dT;
}

// calculate accel x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel) {
	// calculate accel x and y angles
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calculate gyro x, y and z angles in degrees (not necessary for filters)
void calc_gyroAngles(float& angle_x_gyro, float& angle_y_gyro, float& angle_z_gyro) {
	// calculated gyro angles (LSB / 2000000) - initialized with starting angle
	static int64_t angle_x_gyro_scaled = angle_x_gyro * 2000000 * GYRO_SENS;
	static int64_t angle_y_gyro_scaled = angle_y_gyro * 2000000 * GYRO_SENS;
	static int64_t angle_z_gyro_scaled = angle_z_gyro * 2000000 * GYRO_SENS;
	
	// calculate gyro angles (LSB / 2000000)
	angle_x_gyro_scaled += dt * (gx0 + gx);
	angle_y_gyro_scaled += dt * (gy0 + gy);
	angle_z_gyro_scaled += dt * (gz0 + gz);

	// make sure gyro angles do not grow endlessly
	const static int64_t TEMP = (360 * 2000000 * GYRO_SENS);
	angle_x_gyro_scaled = angle_x_gyro_scaled % TEMP;
	angle_y_gyro_scaled = angle_y_gyro_scaled % TEMP;
	angle_z_gyro_scaled = angle_z_gyro_scaled % TEMP;

	// calculate gyro x, y and z angles
	angle_x_gyro = angle_x_gyro_scaled / 2000000 / GYRO_SENS;
	angle_y_gyro = angle_y_gyro_scaled / 2000000 / GYRO_SENS;
	angle_z_gyro = angle_z_gyro_scaled / 2000000 / GYRO_SENS;
}

// print display
void printDisplay(int8_t mode, float velocity_M1, float velocity_M2, float angle_x_KF){
	static const int32_t LCD_REFRESH_TIME = 500000;
	static int32_t LCD_time = 0;
	
	static int8_t mode_old = 0;
	
	static int16_t LCD_temp1, LCD_temp2;
	
	// print menu
	if (mode != mode_old) {
		lcd.clear();
		switch (mode)
		{
			case 1: lcd.setCursor (0, 0);
					lcd.print("        BB-1        ");
					lcd.setCursor (0, 1);
					lcd.print("v:");
					lcd.setCursor (7, 1);
					lcd.print(".");
					lcd.setCursor (12, 1);
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
			case 2:	lcd.setCursor (0, 0);
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
					lcd.setCursor (4, 3);
					lcd.print(".");
					break;
			default: break;
		}
		mode_old = mode;
		
		// reset interrupt status
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();
		first = true;
	}
	
	// print values
	if (LCD_time >= LCD_REFRESH_TIME) {
		LCD_time = 0;
		switch (mode)
		{
			case 1:	/*				
					lcd.setCursor (5, 1);
					LCD_temp2 = round(velocity_M1 / 100);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (velocity_M1 >= 0) {
						lcd.print(" ");
					}
					else {
						lcd.print("-");
					}
					lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
					
					lcd.setCursor (10, 1);
					LCD_temp2 = round(velocity_M2 / 100);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (velocity_M2 >= 0) {
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
					LCD_temp2 = round((float) dt / 100);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (LCD_temp1 < 10) {
						lcd.print(" ");
					}
					lcd.print(LCD_temp1); lcd.moveCursorRight(); lcd.print(LCD_temp2);
					*/
			
					break;
					
					/*	
					static char line_1[7], line_2[6], line_3[4];
					static uint16_t pos;
					
					LCD_temp2 = round(velocity_M1 / 100);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (velocity_M1 >= 0) {
						pos = sprintf(line_1, "%c%u%u", ' ', LCD_temp1, LCD_temp2);
					}
					else {
						pos = sprintf(line_1, "%c%u%u", '-', LCD_temp1, LCD_temp2);
					}
					
					LLCD_temp2 = round(velocity_M2 / 100);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (velocity_M2 >= 0) {
						pos = sprintf(line_1, "%c%u%u", ' ', LCD_temp1, LCD_temp2);
					}
					else {
						pos = sprintf(line_1, "%c%u%u", '-', LCD_temp1, LCD_temp2);
					}
					
					LCD_temp2 = round(angle_x_KF * 10);
					LCD_temp1 = abs(LCD_temp2 / 10);
					LCD_temp2 = abs(LCD_temp2 % 10);
					if (LCD_temp1 < 100) {
						pos = sprintf(line_2, "%c", ' ');
						if (LCD_temp1 < 10) {
							pos = sprintf(line_2 + pos, "%c", ' ');
						}
					}
					if (angle_x_KF >= 0) {
						sprintf(line_2 + pos, "%c%u%u", ' ', LCD_temp1, LCD_temp2);
					}
					else {
						sprintf(line_2 + pos, "%c%u%u", '-', LCD_temp1, LCD_temp2);
					}
					
					LCD_temp2 = round((float) dt / 100);
					LCD_temp1 = LCD_temp2 / 10;
					LCD_temp2 = LCD_temp2 % 10;
					if (LCD_temp1 < 10) {
						pos = sprintf(line_3, "%c", ' ');
					}
					sprintf(line_3 + pos, "%u%u", LCD_temp1, LCD_temp2);
					*/
					
			case 2:	lcd.setCursor (3, 1);
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
					
					lcd.setCursor (3, 3);
					LCD_temp2 = round(D_angle * 100);
					LCD_temp1 = LCD_temp2 / 100;
					LCD_temp2 = LCD_temp2 % 100;
					lcd.print(LCD_temp1); lcd.moveCursorRight();
					if (LCD_temp2 < 10) {
						lcd.print("0");
					}
					lcd.print(LCD_temp2);
					
					break;
					
			default: break;
		}
	}
	else {
		LCD_time += dt;
	}
}

// calibrate MPU
void calibMotion6(const int16_t NUM, const int16_t ACCURACY_ACCEL, const int16_t ACCURACY_GYRO) {
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

		meanMotion6(NUM, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);

		if ((abs(mean_ax) < ACCURACY_ACCEL) &&
		(abs(mean_ax) < ACCURACY_ACCEL) &&
		((abs(mean_az - G_LSB) < ACCURACY_ACCEL) && (mean_az >= -G_LSB * pow(2, ACCEL_RANGE + 1) - 1)) &&
		(abs(mean_gx) < ACCURACY_GYRO) &&
		(abs(mean_gy) < ACCURACY_GYRO) &&
		(abs(mean_gz) < ACCURACY_GYRO)) {
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
void meanMotion6(const int16_t NUM, int16_t& mean_ax, int16_t& mean_ay, int16_t& mean_az, int16_t& mean_gx, int16_t& mean_gy, int16_t& mean_gz)  {
	int32_t sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;

	for (int16_t i = 0; i < NUM; i++)  {
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

	mean_ax = sum_ax / NUM;
	mean_ay = sum_ay / NUM;
	mean_az = sum_az / NUM;
	mean_gx = sum_gx / NUM;
	mean_gy = sum_gy / NUM;
	mean_gz = sum_gz / NUM;
}


// send serial data to Processing
void sendSerial(int32_t dt, float angle_x_gyro, float angle_y_gyro, float angle_z_gyro, float angle_x_accel, float angle_y_accel,
float angle_x_CF, float angle_y_CF, float angle_x_KF, float angle_y_KF) {
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
