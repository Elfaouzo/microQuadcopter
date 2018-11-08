// This #include statement was automatically added by the Particle IDE.
#include "LPS.h"

// This #include statement was automatically added by the Particle IDE.
#include "LIS3MDL.h"

// This #include statement was automatically added by the Particle IDE.
#include "LSM6.h"

/*
Revision 134 - branch for uint8 throttle & mc, second trial
Motor control:  OK.
Wifi link:      OK. 
Timing schedul: OK.
DataLogging     OK. 
Compass         OK.
Yaw fusion      OK: But move fusion to a faster raster. YawRate gyro integration has to be faster.
PPM             OK.
SOC				NOK: Need new PCB. Issue with AnalogRead at high frequency 50ms.
IMU fusion:     OK.
Attitude ctl:   OK.
Altitude ctl:   OK.
ASIS:       
				Define state more carefully:
				state 0: standby -complete stop called fault
				state 1: startup
				state 2: IMU init done
				state 3: ready to go -no throttle
				state 4: running -throttle ON
				state 5: running -take off
				
				Define rasters more carefully:
				IMU and Control -5ms
					IMU raw data readings
					IMU filter and LPF
					Kalman filter: sensor fusion for IMU (ACC+GYRO), ALT (BARO+GYRO), MAG (MAG+GYRO)
					Correction calculation: errors calculation, corrections calculations, saturation of corrections.
					Control mixing: + or X model, merge of static throttle and corrections (IMU, MAG, ALT corrections)           
					Motor actuation
				Wifi 			-20ms
					Wifi command reading
				MAG reading		-50ms
					MAG reading 
				POS reading		-50ms
					Barometer reading and filtering (LPF)
				BATT reading 	-2000ms
				
TODO:
			mcx shall not be a float! but a uint8 instead. Same for the throttle.
			There shall be no assignment of the variant state outside the main loop. 8 cases.
			Improve the display at startup, consider one single display with all info.
			Yaw calculation: The yaw gyro integration was moved from 50ms to 5ms loop, kalman_fo() and update_MAG() have been updated. NO! Revert, result was tested NOK.
			Calibrate PID for altitude control based on simulation.

RELEASE CONTENT:
			Variant PIDAltLevel has been added, PIDAltLevel=2 gives the opportunity of having nested PID for altitude control.
			Variant TAKEOFF_THR_LO added for RC receiver. Set the minimum throttle threshold to trigger a take-off.
			Variant PPM_ATTITUDE_MAX added for RC receiver. Rescale the target attitude request by the RC transmitter by setting the maximum value.
			Variant dataLoggingControlAnalysis has been modified to log only pitch/roll, errorPitch/errorRoll, motors. No yaw, no altitude but logging is longer.
			Datatype of mc1, mc2, mc3, mc4 and throttle has been changed to uint8_t.
			
TO BE TESTED:   
			Attitude control with received only: At fixed altitude, trigger an impulse in roll/pitch for system id.
				- Fixed altitude. OK.
				- Check if functional (start-up only). OK.
				- Enable dataLoggingControlAnalysis instead of dataLoggingPOSAnalysis. OK.
				- Set DiagAngleTolerance=100 deg. OK. 
				- Try to change pitch/roll target, check that it behaves as expected. OK.
				    - Logs show no issues with the set points.
				    - Behaviour seems to be dramatic and the quad difficult to control: controls are too sensitive! reduce effect of the controller by dividing by 5 or 10. OK.
					- Test again.
				- DataLogging shall be at least 10s. Impossible but 7s instead.
				- Flash and update .m script for dataLogging.
				- Trigger impulses in roll and pitch and log.
				
				
			Perform Altitude open loop measurements: 
				- Disable altitude control and introduce impulses in the throttle through RC transmitter.
			
			Consider using nested PID for altitude control
*/


#include "application.h"
#include "math.h"
#include <Wire.h>
#include "i2c_hal.h"

// TEMPORARY VARIANTS
//#define POS_EST
#define ALT_ACC
#define TAKEOFF_AUTO						//auto take off
#define TAKEOFF_AUTO_Ti 800				//in ms
#define TAKEOFF_AUTO_Thr 160				//default thrust at auto take off
#define FLIGHT_AUTO_Thr 130
#define TAKEOFF_THR_LO 5

#define BATT_SOC
#define Thrust2Amp 1				//Consumption per motor at full throttle (throttle is rescalled to 1 here)
#define battCapacity 0.2			//Ah

#define PPM_RECV
#define PPM_TSYNC 10                //in ms
#define PPM_LO 1050
#define PPM_HI 2020
#define PPM_ATTITUDE_MAX 10			//in deg

//#define ADD_GYRO_YAW_CORRECTION
//#define TEST_MOTOR
#define INVERT_CMD                  //Case where the IMU is flipped, INVERT=IMU towards sky

#define PID_yaw_SAT 80
#define YAW_DES_INIT
#define ALT_DES_INIT 0.8
#define ALT_AUTO 1					//Altitude control
#define ALT_LPF
#define PID_alt_SAT 200				//Maximum correction for the altitude AND altitudeSpeed including the static thrust (idle)

#define PID_ATTITUDE_SAT 0.4*255		//between [-1;1]

/*      VARIANTS    */
#define ACC_ONLY_AT_STARTUP
#define STABILITYDETECTION
#define MOTORSATURATIONDETECTION
#define HEART_BEAT

#define CONTROL
#define DIAGNOSTICS                                                            
#define WIFI_LISTEN
#define IMU_ENABLE
#define HEART_BEAT_THR 50

#define DATALOGGING_TCP

#define FastComm1B_ANGLE 2.9
#define FastComm1B_THROTTLE 8.22

#define LED_PIN D7
#define BATT_PIN A0
#define PPM_PIN D4

#define MOTOR_1_PIN TX
#define MOTOR_2_PIN D2
#define MOTOR_3_PIN D3
#define MOTOR_4_PIN RX

#define DiagAngleTolerance 100
#define DiagIMURatThreshold 5                   //  +/- degrees
#define DiagRunTimeRatio 2
#define DiagBaroRat

#define PIDLevel 2
#define PIDAltLevel 1

#define PID_P_roll 1
#define PID_I_roll 5
#define PID_D_roll 1

#define PID2_P_roll 0.8
#define PID2_I_roll 5
#define PID2_D_roll 0.01


#define PID_P_pitch 3				//10
#define PID_I_pitch 10				//0
#define PID_D_pitch 0.5				//0

#define PID2_P_pitch 0.6			//0.3
#define PID2_I_pitch 4				//4
#define PID2_D_pitch 0.012			//0.005

#define PID_P_yaw 2
#define PID_I_yaw 0
#define PID_D_yaw 1

#if ALT_AUTO==1
#define PID_P_alt 0.5
#define PID_I_alt 0.1
#define PID_D_alt 0
#define PID_P_alt_speed 0.6
#define PID_I_alt_speed 0
#define PID_D_alt_speed 0
	#if PIDAltLevel == 2
	#define PID2_P_alt 0.5
	#define PID2_I_alt 0.1
	#define PID2_D_alt 0
	#endif
#else
#define PID_P_alt 0
#define PID_I_alt 0
#define PID_D_alt 0
#define PID_P_alt_speed 0
#define PID_I_alt_speed 0
#define PID_D_alt_speed 0
#endif

//#define TPA_ENABLE
#define PID_THR_LO 140									//Take off threshold under which PID is [0,0,0]
#define PID_COEF_HI 0.7									//By how much the PID has to be damped when THR is =255 compared to when it is PID_THR_LO

#define samplingTimeWifiRead 20
#define samplingTimeCntl 5               
#define samplingTimeMAG 50
#define samplingTimePOS 5
#define samplingTimeBatt 2000
#define samplingTimeDebug 2000

#define AdaptiveKalmanGyro                              //Enable different Kalman gain for when state >=4 or not
#define KalmanGainGyro 0.003						       //Gain for when state >= 4
#define KalmanGainGyroIdle 0.01                       		//Gain for when state != 4    

#define KalmanGainBaroHx 0.01    
#define KalmanGainBaroVx 0.1
#define KalmanGainHx 0.01
#define KalmanGainVx 0.1

#define compassError_LoTh 0.02
#define compassError_LoTh_gain 0.001
#define compassError_AtZero_gain 0.2
#define compassError_AtOne_gain 0.0001

#define Bx 5132.869
#define By -3947.925
#define Bz 3726.067
#define M11 2.101
#define M12 0.058
#define M13 -0.074
#define M21 -0.042
#define M22 2.216
#define M23 -0.223
#define M31 0.056
#define M32 0.293
#define M33 2.212

#define offsetX -3591.5
#define offsetY 1222.5
#define offsetZ 3899

#define factorX 1.0028
#define factorY 1.0127
#define factorZ 1

//#define QualityFactor
#define QfBad 3
#define QfGood 1

//#define dataLoggingIMUAnalysis       
//#define dataLoggingMAGAnalysis      
//#define dataLoggingPOSAnalysis  
#define dataLoggingControlAnalysis
//#define dataLoggingPPMAnalysis
//#define dataLoggingBasicAnalysis
//#define dataLoggingAttitudeAnalysis

#define dataLoggingErrorThreshold 5

#ifdef dataLoggingPPMAnalysis
	#define dataLoggingBufferSize 700
#endif
#ifdef dataLoggingIMUAnalysis
    #define dataLoggingBufferSize 1200
#endif
#ifdef dataLoggingControlAnalysis
    #define dataLoggingBufferSize 800
#endif
#ifdef dataLoggingMAGAnalysis
    #define dataLoggingBufferSize 1000
#endif
#ifdef dataLoggingPOSAnalysis
    #define dataLoggingBufferSize 800
#endif
#ifdef dataLoggingBasicAnalysis
	#define dataLoggingBufferSize 2500
#endif
#ifdef dataLoggingAttitudeAnalysis
	#define dataLoggingBufferSize 500
#endif

#define modelxPitchCoef 1                    

#define AUTOMATIC_MODE
 
#define IMU_POL10V5
#define LPF_IMU

#ifdef IMU_POL10V5
    #define POL10V5_BIAS_COUNT 50                 // count 50 delay 50ms
    #define GYRO_SENSITIVITY 65.5                   // +-250dps over int16_t, i.e 500 dps over 65535 values [-32768; 32767], 32767/250=131.
                                                    // +-500 dps => 32767/500=65.5
#endif

SYSTEM_THREAD(ENABLED);

#ifndef AUTOMATIC_MODE
    SYSTEM_MODE(MANUAL);
#endif

//STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

/*      VARIABLES    */

#ifdef PPM_RECV
void PPM_Read(void);
int PPM_init_ok = 0;
#endif

#ifdef QualityFactor
int QfCntrRoll = 0;
int QfCntrPitch = 0;
float QfTime = 0;
#endif

int state = 1; 
int state_prev;
float rawX, rawY, rawZ, rawXcompass, rawYcompass, rawZcompass;
float rawRollRateGyro, rawPitchRateGyro, rawYawRateGyro;
float altitude;
float temperature;
float pressure;

float filtX, filtY, filtZ, filtXcompass, filtYcompass,filtZcompass, filtRollAcc, filtPitchAcc, filtRollGyro, filtPitchGyro, filtYawGyro, filtRollRateGyro, filtPitchRateGyro, filtYawRateGyro, filtYawCompass;
float rollAcc, pitchAcc, rollRateGyro, pitchRateGyro, yawRateGyro;
float Ax, Ay, Az;
float biasY, biasX;
#ifdef ALT_ACC	
#define LPF_WDN_LENGTH 20									//Acceleration in global coordinates
float filtVz, filtHz, HzBar, filtHz_prev, filtAltitude;			//filtHz is the fast prediction based on HzBar and accelerometer
float biasZ, totalZ, totalBaroAlt, biasBaroAlt;
	#ifdef ALT_LPF
	float HzBarBuff[LPF_WDN_LENGTH];
	int HzBarBuffIdx;
	float HzBarAvg, HzBarFast, HzBarSlow;
	#define KxBarLPF 0.98
	#endif
#endif

#ifdef POS_EST
float pos_x, pos_y;									//positions based on attitude and altitude
float filtHx, filtHy;								//positions based on accelerations and attitude
float filtVx, filtVy;								//Acceleration in global coordinates
float totalX, totalY;
#endif
int indexBuffer;
float roll, pitch, yaw, roll_prev, pitch_prev;
#ifdef ACC_ONLY_AT_STARTUP
float initRoll, initPitch, initYaw;
#endif

#ifdef YAW_DES_INIT									//If this is disabled the desired_yaw will be 0 deg at startup.
float desired_yaw_init;
#endif

#ifdef LPF_IMU										//LPF on IMU data before kalman
float circBuff_pitchAcc[3];
float circBuff_rollAcc[3];
float circBuff_pitchGyroRate[3];
float circBuff_rollGyroRate[3];

float LPFCoefRollAcc[] = {0.08,0.25,0.25,0.2};
float LPFCoefPitchAcc[] = {0.08,0.25,0.25,0.2};
float LPFCoefRollGyroRate[] = {0.2,0.2,0.2,0.2};
float LPFCoefPitchGyroRate[] = {0.1,0.2,0.2,0.2};
#endif

float desired_roll, desired_pitch, desired_yaw, desired_alt;
float corr_roll, corr_pitch, corr_yaw, corr_alt, corr_alt_speed, corr_alt_total, corr2_roll, corr2_pitch;
float error_roll, error_pitch, error_yaw, error_alt, error_alt_speed;
#if PIDAltLevel == 2
float error2_alt, corr2I_alt, corr2_alt, error2Prev_alt;
#endif

float biasRollRate, biasPitchRate, biasYawRate;
int16_t mc1, mc2, mc3, mc4, throttle, staticThrottle;
int16_t takeOffThrottle, flightThrottle;
uint16_t throttle1;
uint8_t throttle2;
float totalGyroYawRate, totalGyroRollRate, totalGyroPitchRate, totalAccPitch, totalAccRoll;

float kxGyro = KalmanGainGyro;
float kxCompass;
float compassMagnitude;
float compassError;

#ifdef IMU_POL10V5
LSM6 imu;
LIS3MDL mag;
LPS ps;
#endif

#ifdef BATT_SOC
float battVoltage;
float battOCV;
float battSOC;
float battCoulombCounterSOC;
int CoulombCountTs = samplingTimeCntl;
const float OCV_V[] = {3.2, 3.60, 3.65, 3.70, 3.72, 3.77, 3.80, 3.90, 3.95, 4.00, 4.21};
const float OCV_S[] = {0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00};
const int OCV_size = 11;
float OCV_offset = 0.13*0;	//Photon drains 80mA in average

const float OCV_ThrTo[]={235, 235, 225, 225, 220, 210, 200, 200, 190, 190, 180};			//Static throttle at take-off
const float OCV_ThrFl[]={225, 225, 210, 200, 170, 160, 150, 140, 140, 130, 120};			//Static throttle during flight

const float OCV_TOTiOut[]={1000., 1000., 1000., 800., 800., 800., 800., 800., 800., 800., 800.};			//Take-off timeout in ms
#endif


// PID controller
double Kp[] = {PID_P_roll, PID_P_pitch, PID_P_yaw, PID_P_alt, PID_P_alt_speed};       
double Ki[] = {PID_I_roll, PID_I_pitch, PID_I_yaw, PID_I_alt, PID_I_alt_speed};           
double Kd[] = {PID_D_roll, PID_D_pitch, PID_D_yaw, PID_D_alt, PID_D_alt_speed};    

#if PIDAltLevel == 2
double K2p[] = {PID2_P_roll, PID2_P_pitch, PID2_P_alt};           
double K2i[] = {PID2_I_roll, PID2_I_pitch, PID2_I_alt};           
double K2d[] = {PID2_D_roll, PID2_D_pitch, PID2_D_alt};    
#else
double K2p[] = {PID2_P_roll, PID2_P_pitch};           
double K2i[] = {PID2_I_roll, PID2_I_pitch};           
double K2d[] = {PID2_D_roll, PID2_D_pitch};    
#endif	
float errorPrev_roll, corrI_roll, errorPrev_pitch, corrI_pitch, errorPrev_yaw, corrI_yaw, errorPrev_alt, corrI_alt, corrI_alt_speed, errorPrev_alt_speed;
float error2Prev_roll, corr2I_roll, error2Prev_pitch, corr2I_pitch;
float droll, dpitch;

#ifdef MOTORSATURATIONDETECTION
int8_t saturationMotor[] = {0,0,0,0};
#endif

//Timers
uint32_t currentTimeCntl, currentTimeWifiRead, currentTimeDebug, currentTimeLogging, currentTimeMAG, currentTimePOS;
uint16_t currentTimeCntlTmp, currentTimeMAGTmp, currentTimeWifiReadTmp, currentTimePOSTmp;
uint16_t CTL_timing_MAX, CTL_timing_MIN;
uint32_t scaleTick;
#ifdef TAKEOFF_AUTO
uint32_t currentTimeTO, currentTimeTOTmp;
uint32_t timeOutTO;
#endif

#ifdef PPM_RECV
long currentTimePPM;
long currentTimePPM_temp;
uint8_t PPM_state = 0;
long PPM_CHN_DATA[10];
#endif

#ifdef BATT_SOC
uint16_t currentTimeBattTmp;
uint32_t currentTimeBatt;
#endif

//WIFI setup
int tcp_port;
char tmp0;
char starter;
String variable_CAN, value_CAN, lst, local_IP;
byte client_connected;
TCPServer server = TCPServer(1001);
TCPClient client;
String returnCariage = "\n";
String tmp; 
#ifdef HEART_BEAT
uint8_t heart_beat_count;
#endif

bool errorLog;
bool errorRunTime;
bool errorIMURationality;
bool errorStability;
int DiagBaroRatBuff;

#ifdef dataLoggingBasicAnalysis                     //Every 50ms atm
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float pitchLog[dataLoggingBufferSize];
  float rollLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif
#ifdef dataLoggingIMUAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float pitchLog[dataLoggingBufferSize];
  float rollLog[dataLoggingBufferSize];
  float filtPitchAccLog[dataLoggingBufferSize];
  float filtRollAccLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  float filtRollGyroRateLog[dataLoggingBufferSize];
  float filtPitchGyroRateLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif
#ifdef dataLoggingAttitudeAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float pitchLog[dataLoggingBufferSize];
  float rollLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  uint8_t mc1Log[dataLoggingBufferSize];
  uint8_t mc2Log[dataLoggingBufferSize];
  uint8_t mc3Log[dataLoggingBufferSize];
  uint8_t mc4Log[dataLoggingBufferSize];
  float error_rollLog[dataLoggingBufferSize];
  float error_pitchLog[dataLoggingBufferSize];
  float corr_rollLog[dataLoggingBufferSize];
  float corr_pitchLog[dataLoggingBufferSize];
  float drollLog[dataLoggingBufferSize];
  float dpitchLog[dataLoggingBufferSize];
  float corr2_rollLog[dataLoggingBufferSize];
  float corr2_pitchLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif
#ifdef dataLoggingControlAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float pitchLog[dataLoggingBufferSize];
  float rollLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  uint8_t mc1Log[dataLoggingBufferSize];
  uint8_t mc2Log[dataLoggingBufferSize];
  uint8_t mc3Log[dataLoggingBufferSize];
  uint8_t mc4Log[dataLoggingBufferSize];
  float error_rollLog[dataLoggingBufferSize];
  float error_pitchLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif
#ifdef dataLoggingPPMAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float pitchLog[dataLoggingBufferSize];
  float rollLog[dataLoggingBufferSize];
  float yawLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  uint8_t mc1Log[dataLoggingBufferSize];
  uint8_t mc2Log[dataLoggingBufferSize];
  uint8_t mc3Log[dataLoggingBufferSize];
  uint8_t mc4Log[dataLoggingBufferSize];
  float altitudeLog[dataLoggingBufferSize];
  float desired_rollLog[dataLoggingBufferSize];
  float desired_pitchLog[dataLoggingBufferSize];
  float desired_yawLog[dataLoggingBufferSize];
  float stateLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif
#ifdef dataLoggingMAGAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float yawLog[dataLoggingBufferSize];
  float filtYawGyroLog[dataLoggingBufferSize];
  float filtYawRateGyroLog[dataLoggingBufferSize];
  float filtYawCompassLog[dataLoggingBufferSize];
  uint8_t throttleLog[dataLoggingBufferSize];
  float compassErrorLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif

#ifdef dataLoggingPOSAnalysis
struct dataLog{
  uint16_t stampTime[dataLoggingBufferSize];  
  float rollLog[dataLoggingBufferSize];
  float pitchLog[dataLoggingBufferSize];
  #ifdef POS_EST
  float filtHxLog[dataLoggingBufferSize];
  float filtHyLog[dataLoggingBufferSize];
  float pos_xLog[dataLoggingBufferSize];
  float pos_yLog[dataLoggingBufferSize];
  float AxLog[dataLoggingBufferSize];
  float AyLog[dataLoggingBufferSize];
  float filtXLog[dataLoggingBufferSize];
  float filtYLog[dataLoggingBufferSize];
  #endif
  float filtHzLog[dataLoggingBufferSize];
  uint8_t mc1Log[dataLoggingBufferSize];
  uint8_t mc2Log[dataLoggingBufferSize];
  uint8_t mc3Log[dataLoggingBufferSize];
  uint8_t mc4Log[dataLoggingBufferSize];
  float filtZLog[dataLoggingBufferSize];
  float HzBarLog[dataLoggingBufferSize];
  uint8_t staticThrottleLog[dataLoggingBufferSize];
  float corr_alt_totalLog[dataLoggingBufferSize];
  uint16_t idx;
};
#endif

#ifdef DATALOGGING_TCP
struct dataLog buffer;
#endif

void setup() {     
  state = 1;                                    //State 1 = Init
  motors_init();
  local_init();
   
  #ifdef BATT_SOC
	Batt_Read();
	delay(1500);
	String socDisplay = "SoC (Ocv) [%]: " + String(battSOC) + " - OCV [V]: " + String(battOCV);
	Particle.publish("SYS", socDisplay);
	delay(1500);
	updateDynamicThrottles(battSOC);		//updates takeOffThrottle
	updateDynamicTOTimeOut(battSOC);
	socDisplay = "SoC (Ocv) [%]: " + String(battSOC) + " - ThrToff/ThrFl: " + String(takeOffThrottle)+ "/" + String(flightThrottle) + " - TOTiOut [ms]: " + String(timeOutTO);
	Particle.publish("SYS", socDisplay);
	delay(1500);
  #else
	takeOffThrottle=TAKEOFF_AUTO_Thr;	
	flightThrottle=FLIGHT_AUTO_Thr;
	timeOutTO = TAKEOFF_AUTO_Ti;
  #endif

  currentTimeCntl = System.ticks();
  currentTimeWifiRead = System.ticks();
  currentTimeDebug = System.ticks();
  currentTimeMAG = System.ticks();
  currentTimePOS = System.ticks();
  scaleTick = System.ticksPerMicrosecond()*1000;

  #ifndef AUTOMATIC_MODE
    Particle.connect();
    Particle.process();
    delay(500);
  #endif
  if(state == 1)
  {
    state = 2;                                    //State 2 = IMU init
  }
  init_IMU();
  if(state!=0)
  {
	calibrate_IMU();
  }
  #ifndef AUTOMATIC_MODE
    Particle.process();
  #endif
  delay(500);
  uint32_t FreeMem = System.freeMemory();
  String memDisplay = "Available memory: " + String(FreeMem);
  Particle.publish("SYS",memDisplay);
  delay(1000);
  String tempDisplay = "Board temperature: " + String(temperature);
  Particle.publish("SYS",tempDisplay);
  #ifndef AUTOMATIC_MODE
    Particle.process();
  #endif
  
  if(state == 2)
  {
	state = 3;                                    //State 3 = active idle
  }
  #ifndef AUTOMATIC_MODE
    Particle.process();
  #endif
  if(state == 3 || state == 4)
  {
	delay(1000);
	Particle.publish("SYS","State active");
  }
  else
  {
	  delay(2500);
	  Particle.publish("SYS","Init failed!");
  }
  #ifndef AUTOMATIC_MODE
    Particle.process();
  #endif
  #ifndef AUTOMATIC_MODE
    Particle.publish("SYS","Disconnecting from the cloud...");
    Particle.process();
    delay(500);
    Particle.disconnect();
  #endif
  
  #ifdef DIAGNOSTICS
    //check_IMU_rationality();
    delay(200);
    //check_IMU_rationality();
  #endif
  
  #ifdef PPM_RECV
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(PPM_PIN, PPM_Read, RISING);
  currentTimePPM = micros();
  #endif
  if(state == 3 || state == 4)
  {
	LedStateON();
  }
  else
  {
	  delay(2500);
	  Particle.publish("SYS","Init failed!");
  }
  
  
  currentTimeCntl = System.ticks();
  currentTimeWifiRead = System.ticks();
  currentTimeDebug = System.ticks();
  currentTimeLogging = System.ticks();
  currentTimeMAG = System.ticks();
  currentTimePOS = System.ticks();
  #ifdef BATT_SOC
  currentTimeBatt = System.ticks();
  #endif
}

void loop() {
	#ifdef BATT_SOC
		currentTimeBattTmp = (System.ticks() - currentTimeBatt)/scaleTick;
        if(currentTimeBattTmp >= samplingTimeBatt)
        {
            //Batt_Read();
            currentTimeBatt = System.ticks();
        }
	#endif
    #ifndef TEST_MOTOR
    #ifdef WIFI_LISTEN
        currentTimeWifiReadTmp = (System.ticks() - currentTimeWifiRead)/scaleTick;
        if(currentTimeWifiReadTmp >= samplingTimeWifiRead)
        {
			state_prev=state;
            get_desired_param_1B();
            currentTimeWifiRead = System.ticks();
			#ifdef TAKEOFF_AUTO
			if(state_prev==3 && state==4)
			{
			state=5;
			currentTimeTO=System.ticks();
			}
			#endif
        }
    #endif
	#ifdef TAKEOFF_AUTO
		if(state==5)
		{
			currentTimeTOTmp = (System.ticks() - currentTimeTO)/scaleTick;
			if(currentTimeTOTmp >= timeOutTO)			//timeout of take-off
			{	
				desired_alt=filtHz;						//Set the altitude in the end of the take-off as the target altitude
				state=4;
			}
		}
	#endif
	#if defined(POS_EST) || defined(ALT_ACC)
		currentTimePOSTmp = (System.ticks() - currentTimePOS)/scaleTick;
        if(currentTimePOSTmp >= samplingTimePOS)
        {
            update_POS();
			
			#ifdef dataLoggingPOSAnalysis
            if(!errorLog)                 //Logging every 50ms
            {
                if(state >= 3)
                {
                    updateLog();
                }
            }
			#endif
            currentTimePOS = System.ticks();
        }
	#endif
    #ifdef IMU_ENABLE
        currentTimeMAGTmp = (System.ticks() - currentTimeMAG)/scaleTick;
        if(currentTimeMAGTmp >= samplingTimeMAG)
        {
            update_MAG();
			update_ALT();
            #if defined(dataLoggingMAGAnalysis) || defined(dataLoggingBasicAnalysis)
            if(!errorLog)                 //Logging every 50ms
            {
                if(state >= 3)
                {
                    updateLog();
                }
            }
			#endif
            currentTimeMAG = System.ticks();
        }
    #endif
    #ifdef CONTROL
        currentTimeCntlTmp = (System.ticks() - currentTimeCntl)/scaleTick;
        if(currentTimeCntlTmp >= samplingTimeCntl)
        {
            update_IMU();
			kalman_fo();
            #ifdef STABILITYDETECTION
			if(state >= 4)
			{
				check_stability();
			}
            #endif
            if(state >= 3)                    
            {
                update_correction_parameters();
                update_control_mixing();                  
                update_motors_control();
				#ifdef BATT_SOC
				//coulombCount();
				#endif
            }
            else
            {
                motors_stop();
            }
            #if defined(dataLoggingIMUAnalysis) || defined(dataLoggingControlAnalysis) || defined(dataLoggingPPMAnalysis) || defined(dataLoggingAttitudeAnalysis)
            if(!errorLog)                 //Logging every 5ms
            {
                if(state >= 3)
                {
                    updateLog();
					#ifdef QualityFactor
					updateQf();
					#endif
                }
            }
			#endif
            CTL_timing_MAX = max(currentTimeCntlTmp,CTL_timing_MAX); 
            CTL_timing_MIN = min(currentTimeCntlTmp,CTL_timing_MIN);
            currentTimeCntl = System.ticks();
        }
    #endif
    #ifdef DIAGNOSTICS
        if((System.ticks() - currentTimeDebug)/scaleTick >= samplingTimeDebug)
        {
            //publish_IMU_Bias();
            //publish_IMU_Raw();
            //publish_Kalman();
            //publish_corrParamP();
            //publish_mc();
            //publish_debugKalman();
            //transmit_flightParam();
            //transmit_debugKalman();
            //publish_throttle();
            //publish_Compass();
            //publish_IMU_RawCompass();
            //publish_IMU_Filt();
            //publish_flightParam();
            /*if(state >= 3)
            {
                check_RunTime();                        //Diagnostic
            }*/
            //publish_altimeter();
			//publish_battStatus();
			throttle1=(uint16_t)(throttle);
			throttle2=(uint8_t)(throttle1);
			Particle.publish(String(throttle) + " - " + String(throttle1) + " - " + String(throttle2));
            currentTimeDebug = System.ticks();
        }
    #endif
    #endif
    
    #ifdef TEST_MOTOR
    testMotor(1);
    delay(5000);
    motors_stop();
    delay(5000);
    #ifndef AUTOMATIC_MODE
    Particle.connect();
    #endif
    #endif
}

float TPA_coefficient()
{
	float coefficient = 0;
	
	if(throttle < PID_THR_LO - 10)
	{
		coefficient = 0;
	}
	else
	{
		coefficient = 1 - (throttle-PID_THR_LO)*(1-PID_COEF_HI)/(255-PID_THR_LO);
	}
	return coefficient;
}

void local_init()
{
	
  #ifdef DATALOGGING_TCP
  buffer.idx = 0;
  #endif
  #ifdef BATT_SOC
  pinMode(BATT_PIN, INPUT);
  #endif
  pinMode(LED_PIN, OUTPUT);
  starter = 0;
  value_CAN = "";
  variable_CAN = "";
  lst = "";
  tcp_port = 1001;
  desired_roll = 0;
  desired_pitch = 0;
  desired_yaw = 0;

  errorPrev_roll = 0; 
  corrI_roll = 0; 
  errorPrev_pitch = 0; 
  corrI_pitch = 0;
  mc1 = 0;
  mc2 = 0;
  mc3 = 0;
  mc4 = 0;
  
  client_connected = 0;
  errorLog = 0;
  errorStability = 0;
  errorIMURationality = 0;
  errorIMURationality = 0;
}


////////////////// DIAGNOSTIC SECTION //////////////////////

void check_stability()
{
    
    if(abs(roll - desired_roll) > DiagAngleTolerance)
    {
        standby();
        if(!errorStability)
        {
            Particle.publish("DIAG","INSTABILITY DETECTED: Roll " + String(roll - desired_roll) + " deg");
            errorStability = 1;
        }
        
    }
    if(abs(pitch - desired_pitch) > DiagAngleTolerance)
    {
        standby();
        if(!errorStability)
        {
            Particle.publish("DIAG","INSTABILITY DETECTED: Pitch " + String(pitch - desired_pitch)+ " deg");
            errorStability = 1;
        }
    }
}

void check_IMU_rationality()
{
    //Run once at startup ONLY
    if(abs(roll) > DiagIMURatThreshold)
    {
        standby();
        if(!errorIMURationality)
        {
            Particle.publish("DIAG","IMU RATIONALITY: Roll " + String(roll) + " deg");
            errorIMURationality = 1;
        }
    }
    if(abs(pitch) > DiagIMURatThreshold)
    {
        standby();
        if(!errorIMURationality)
        {
            Particle.publish("DIAG","IMU RATIONALITY: Pitch " + String(pitch)+ " deg");
            errorIMURationality = 1;
        }
    }
}

void check_RunTime()
{
    //Make sure the timing is respected
    if(CTL_timing_MAX > DiagRunTimeRatio*samplingTimeCntl)
    {
        if(!errorRunTime)
        {
            standby();
            Particle.publish("DIAG","PERFORMANCE: CTL " + String(CTL_timing_MAX) + " ms");
            errorRunTime = 1;
        }
    }
}

////////////////// CONTROL SECTION //////////////////////


void standby()
{
  motors_stop();
  LedStateOFF();
  if(state != 0)
  {
    #ifndef AUTOMATIC_MODE
        Particle.connect();
    #endif
        
    #ifndef AUTOMATIC_MODE
        Particle.process();
        delay(1000);
    #endif
    
    #ifdef DATALOGGING_TCP
        transmit_DataLogChunk();
    #endif
	delay(500);
	publish_timings();
	#ifdef QualityFactor
	delay(500);
	int QfRoll = QfCntrRoll/(QfTime/(samplingTimeCntl*0.001));
	int QfPitch = QfCntrPitch/(QfTime/(samplingTimeCntl*0.001));
	Particle.publish("DIAG","Qf Roll: " + String(QfRoll) + ",Qf Pitch:" + String(QfPitch));
	QfCntrRoll = 0;
	QfCntrPitch = 0;
	QfTime = 0;
	#endif
    #ifdef MOTORSATURATIONDETECTION
	delay(500);
    Particle.publish("DIAG","Motor saturation: " + String(saturationMotor[0]) + "," + saturationMotor[1] + "," + saturationMotor[2] + "," + saturationMotor[3]);
    #endif
  }
  #ifndef AUTOMATIC_MODE
    if (Particle.connected() == false) 
    {
        Particle.connect();
    }
    Particle.process();
    delay(1000);
  #endif
  state = 0;                                //State 0 = Standby
  throttle = 0;
}

void unlock_standby()
{
  if(state == 0)
  {
    Particle.publish("SYS","State: Active");
    #ifndef AUTOMATIC_MODE
    delay(500);
    Particle.publish("SYS","Disconnecting from the cloud...");
    Particle.process();
    Particle.disconnect();
    #endif
  }
  state = 3;                            //State 3 = active
  errorStability = 0;
  LedStateON();
}

void update_ALT()
{
    temperature = ps.readTemperatureC();
	pressure = ps.readPressureMillibars();
    altitude = ps.pressureToAltitudeMeters(pressure);
	filter_ALT();
}

void filter_ALT()
{
	HzBar = altitude - biasBaroAlt; 
	#ifdef ALT_LPF
		HzBarAvg-=HzBarBuff[HzBarBuffIdx];
		HzBarBuff[HzBarBuffIdx]=altitude;
		HzBarAvg+=HzBarBuff[HzBarBuffIdx];
		HzBarBuffIdx++;
		if(HzBarBuffIdx==LPF_WDN_LENGTH)
		{
			HzBarBuffIdx=0;
		}
		HzBarFast=HzBarAvg/LPF_WDN_LENGTH;
		HzBarSlow=HzBarSlow*KxBarLPF + HzBarFast*(1-KxBarLPF);
		if(state<=3)
		{
			biasBaroAlt=HzBarFast;
		}
	#endif
}

void update_MAG()
{
    #ifdef IMU_POL10V5
    mag.read();
    
    float deltaT = samplingTimeMAG*0.001;

	//COMPASS
    rawXcompass = mag.m.x;
    rawYcompass = mag.m.y;
    rawZcompass = mag.m.z;
    AdjustCompass();					//update filtXcompass, filtYcompass, filtZcompass
	if(compassMagnitude != 0)
	{
		compassError = abs((sqrt(filtXcompass*filtXcompass+filtYcompass*filtYcompass+filtZcompass*filtZcompass) - compassMagnitude)/compassMagnitude);    //close to 0 if trustable
		if(compassError < 0)
		{
		    compassError = 0;
		}
		if(compassError > 1)
		{
		    compassError = 1;
		}
		
		float slope1 = (compassError_LoTh_gain - compassError_AtZero_gain)/compassError_LoTh;
		float slope2 = (compassError_AtOne_gain - compassError_LoTh_gain)/(1-compassError_LoTh);
		if(compassError < compassError_LoTh)
		{
			kxCompass = compassError_AtZero_gain + slope1*compassError;
		}
		else
		{
			kxCompass = compassError_LoTh_gain + slope2*(compassError - compassError_LoTh);
		}

	}

		//GYRO
	filtYawGyro = yaw - deltaT*filtYawRateGyro;
	if(filtYawGyro > 180)
	{
	    filtYawGyro = filtYawGyro-360;
	}
	if(filtYawGyro < -180)
	{
	    filtYawGyro = filtYawGyro+360;
	}
	
	int16_t XH = filtXcompass*cos(pitch*PI/180) + filtYcompass*sin(pitch*PI/180)*sin(roll*PI/180) + filtZcompass*sin(pitch*PI/180)*cos(roll*PI/180);
    int16_t YH = filtYcompass*cos(roll*PI/180) + filtZcompass*sin(roll*PI/180);
    filtYawCompass = atan2(YH, XH)*180/PI;  
	yaw = filtYawGyro*(1-kxCompass) + filtYawCompass*kxCompass;
    #endif
}

void update_IMU()
{
  #ifdef IMU_ENABLE
    #ifdef IMU_POL10V5
        imu.read();
        rawRollRateGyro = imu.g.x;      //in deg/s
        rawPitchRateGyro = imu.g.y;
        rawYawRateGyro = imu.g.z;
        rawY = imu.a.y;                 //in m/s^2
        rawX = imu.a.x;
        rawZ = imu.a.z;

    #endif
    filter_IMU();
  #else
  rawZ = 0;
  rawX = 0; 
  rawY = 0; 
  rawRollRateGyro = 0;
  rawPitchRateGyro = 0;
  rawYawRateGyro = 0;
  #endif
}

void filter_IMU()
{
  #ifdef IMU_ENABLE
    int gyroSensitivity = 1;
    int accSensitivity = 1;
	#ifdef IMU_POL10V5
        gyroSensitivity = GYRO_SENSITIVITY;
        accSensitivity = 1;
    #endif
        
    filtX = rawX / accSensitivity;
    filtY = rawY / accSensitivity;
    filtZ = -1*rawZ / accSensitivity;
	
	rollAcc = -asin((float)filtY/sqrt(filtX*filtX+filtZ*filtZ+filtY*filtY))*180/PI;
	pitchAcc = asin((float)filtX/sqrt(filtY*filtY+filtZ*filtZ+filtX*filtX))*180/PI;
	filtRollAcc = rollAcc;
    filtPitchAcc = pitchAcc;
    
    rollRateGyro = (rawRollRateGyro - biasRollRate)/gyroSensitivity;
    pitchRateGyro = (rawPitchRateGyro - biasPitchRate)/gyroSensitivity;
	yawRateGyro = (rawYawRateGyro - biasYawRate)/gyroSensitivity;
	filtRollRateGyro = rollRateGyro;
	filtPitchRateGyro = pitchRateGyro;
    filtYawRateGyro = yawRateGyro;

	#ifdef LPF_IMU
	if(state >= 3)																			//Dont run this during calibration where state=2
	{
		filtRollAcc=LPFCoefRollAcc[3]*circBuff_rollAcc[2]+LPFCoefRollAcc[2]*circBuff_rollAcc[1]+LPFCoefRollAcc[1]*circBuff_rollAcc[0]+LPFCoefRollAcc[0]*rollAcc;
		circBuff_rollAcc[2] = circBuff_rollAcc[1]; circBuff_rollAcc[1] = circBuff_rollAcc[0]; circBuff_rollAcc[0] = filtRollAcc;
		
		filtPitchAcc=LPFCoefPitchAcc[3]*circBuff_pitchAcc[2]+LPFCoefPitchAcc[2]*circBuff_pitchAcc[1]+LPFCoefPitchAcc[1]*circBuff_pitchAcc[0]+LPFCoefPitchAcc[0]*pitchAcc;
		circBuff_pitchAcc[2] = circBuff_pitchAcc[1]; circBuff_pitchAcc[1] = circBuff_pitchAcc[0]; circBuff_pitchAcc[0] = filtPitchAcc;
		
		filtRollRateGyro=LPFCoefRollGyroRate[3]*circBuff_rollGyroRate[2]+LPFCoefRollGyroRate[2]*circBuff_rollGyroRate[1]+LPFCoefRollGyroRate[1]*circBuff_rollGyroRate[0]+LPFCoefRollGyroRate[0]*rollAcc;
		circBuff_rollGyroRate[2] = circBuff_rollGyroRate[1]; circBuff_rollGyroRate[1] = circBuff_rollGyroRate[0]; circBuff_rollGyroRate[0] = rollRateGyro;
		
		filtPitchRateGyro=LPFCoefPitchGyroRate[3]*circBuff_pitchGyroRate[2]+LPFCoefPitchGyroRate[2]*circBuff_pitchGyroRate[1]+LPFCoefPitchGyroRate[1]*circBuff_pitchGyroRate[0]+LPFCoefPitchGyroRate[0]*rollAcc;
		circBuff_pitchGyroRate[2] = circBuff_pitchGyroRate[1]; circBuff_pitchGyroRate[1] = circBuff_pitchGyroRate[0]; circBuff_pitchGyroRate[0] = pitchRateGyro;
	}
	#endif
	
  #endif
}

void kalman_fo()
{
	float deltaT = samplingTimeCntl*0.001;
	filtRollGyro = roll - deltaT*filtRollRateGyro;                                                                  
    filtPitchGyro = pitch - deltaT*filtPitchRateGyro;

	
#ifdef ADD_GYRO_YAW_CORRECTION
	filtRollGyro += filtPitchGyro*sin(filtYawRateGyro*deltaT*PI/(GYRO_SENSITIVITY*180));                                                         
    filtPitchGyro -= filtRollGyro*sin(filtYawRateGyro*deltaT*PI/(GYRO_SENSITIVITY*180));   
#endif

#ifdef AdaptiveKalmanGyro
	if(state>=4)
	{
		kxGyro = KalmanGainGyro;
	}
	else
	{
		kxGyro = KalmanGainGyroIdle;	
	}
#endif
    pitch = filtPitchGyro*(1-kxGyro) + filtPitchAcc*kxGyro ;        
    roll = filtRollGyro*(1-kxGyro) + filtRollAcc*kxGyro;
	
	#ifdef ALT_ACC
	float ToMS2=2.5e-3;
	Az = -(filtZ-biasZ)*ToMS2*cos(pitch*3.1415/180)*cos(roll*3.1415/180);
	Ay = (filtY-biasY)*ToMS2*cos(pitch*3.1415/180)*cos(roll*3.1415/180);
	Ax = (filtX-biasX)*ToMS2*cos(pitch*3.1415/180)*cos(roll*3.1415/180);
	
	filtVz=filtVz+Az*deltaT; 
	filtHz=filtHz+filtVz*deltaT; 
	
	#ifdef POS_EST
	filtVx+=Ax*deltaT; 
	filtHx+=filtVx*deltaT;
	filtVy+=Ay*deltaT; 
	filtHy+=filtVy*deltaT;
	#endif

	#ifdef TAKEOFF_AUTO
	if(state==4)			//During flight only.
	{
		filtVz = filtVz - KalmanGainBaroVx*(filtHz - HzBar);
		filtHz = filtHz - KalmanGainBaroHx*(filtHz - HzBar);
	}
	if(state<=3)			//during standby we do not calculate any altitude, always 0. Reference is taken at take-off
	{
		filtVz=0;
		filtHz=0;
	}
							//when state==5, only the accelerometer is considered.
	#else
	filtVz = filtVz - KalmanGainBaroVx*(filtHz - HzBar);
	filtHz = filtHz - KalmanGainBaroHx*(filtHz - HzBar);
	#endif
	
	#endif
	
}

void update_POS()
{
	#ifdef POS_EST	
	pos_x += (filtHz - filtHz_prev)/(tan(pitch*3.1415/180));
	pos_y += (filtHz - filtHz_prev)/(tan(roll*3.1415/180));
	
	filtVx -= KalmanGainVx*(filtHx - pos_x);
	filtHx -= KalmanGainHx*(filtHy - pos_y);
	
	filtHz_prev = filtHz;
	#endif
}

void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}



void init_IMU()
{
    #ifdef IMU_ENABLE
        #ifdef IMU_POL10V5
        Wire.begin(); 
        Wire.setSpeed(CLOCK_SPEED_400KHZ);
        if (!ps.init())
        {
            #ifndef AUTOMATIC_MODE
                if (Particle.connected() == false) 
                {
                    Particle.connect();
                }
				delay(2500);
                Particle.publish("IMU","ALT init failed!");
                Particle.process();
                delay(1000);
			#else
				delay(2500);
				Particle.publish("IMU","ALT init failed!");
                delay(1000);
            #endif
            standby();
            
        }
        if (!mag.init())
        {
            #ifndef AUTOMATIC_MODE
                if (Particle.connected() == false) 
                {
                    Particle.connect();
                }
				delay(2500);
                Particle.publish("IMU","MAG init failed!");
                Particle.process();
                delay(1000);
			#else
				delay(2500);
				Particle.publish("IMU","MAG init failed!");
				delay(1000);
            #endif
            standby();
            
        }
        if (!imu.init())
        {
            #ifndef AUTOMATIC_MODE
                if (Particle.connected() == false) 
                {
                    Particle.connect();
                }
				delay(2500);
                Particle.publish("IMU","GYRO/ACC init failed!");
                Particle.process();
                delay(1000);
			#else
				delay(2500);
				Particle.publish("IMU","GYRO/ACC init failed!");
				delay(1000);
            #endif
            standby();
        }
		if(state != 0)
		{
			imu.enableDefault_Quad();
			mag.enableDefault();
			ps.enableDefault_Quad();
        
			pressure = ps.readPressureMillibars();
			altitude = ps.pressureToAltitudeMeters(pressure);
			temperature = ps.readTemperatureC();
		}
   
        #endif
        
    #endif
}


void AdjustCompass()    
{
  filtXcompass = (int16_t)(rawXcompass - offsetX)*factorX;
  filtYcompass = (int16_t)(rawYcompass - offsetY)*factorY;
  filtZcompass = (int16_t)(rawZcompass - offsetZ)*factorZ;
}

// All the bias are calculated here based on raw data.
void calibrate_IMU()
{
  update_IMU();
  #ifdef IMU_ENABLE
    int bias_count;
    
    #ifdef IMU_POL10V5
    bias_count = POL10V5_BIAS_COUNT;
    #endif
        //BIAS CALCULATION
        int i;
        for (i = 0; i < bias_count; i++) {
            update_IMU();
			update_MAG();
			update_ALT();
            totalGyroRollRate +=  rawRollRateGyro;
            totalGyroPitchRate +=  rawPitchRateGyro;
            totalGyroYawRate += rawYawRateGyro;
		#ifdef ACC_ONLY_AT_STARTUP
			totalAccRoll +=filtRollAcc;
			totalAccPitch +=filtPitchAcc;
		#endif
		#ifdef ALT_ACC
			#ifdef DiagBaroRat
			if(altitude == 0)
			{
				DiagBaroRatBuff++;
			}				
			#endif
			totalBaroAlt += altitude;
			totalZ += filtZ;
		#endif
		#ifdef POS_EST
			totalX += filtX;
			totalY += filtY;
		#endif
            delay(50);
        }
        biasRollRate = totalGyroRollRate / bias_count;
        biasPitchRate = totalGyroPitchRate / bias_count;
        biasYawRate = totalGyroYawRate / bias_count;
		#ifdef ACC_ONLY_AT_STARTUP
		initRoll = totalAccRoll / bias_count;
		initPitch = totalAccPitch / bias_count;
		roll = initRoll;
		pitch = initPitch;
		#endif
		
		#ifdef ALT_ACC
		biasBaroAlt = totalBaroAlt / bias_count;
		HzBar = altitude - biasBaroAlt; 
			#ifdef DiagBaroRat
			if(DiagBaroRatBuff == bias_count)
			{
				#ifndef AUTOMATIC_MODE
				Particle.process();
				#endif
				delay(1000);
				Particle.publish("Diag","Barometer OFF");
				#ifndef AUTOMATIC_MODE
				Particle.process();
				#endif
				standby();
			}				
			#endif
			
		biasZ = totalZ / bias_count;
		desired_alt = ALT_DES_INIT;
		#endif
		#ifdef POS_EST
		biasX = totalX / bias_count;
		biasY = totalY / bias_count;
		#endif
		
        update_MAG();
		compassMagnitude = sqrt(filtXcompass*filtXcompass+filtYcompass*filtYcompass+filtZcompass*filtZcompass);
		yaw = filtYawCompass;
		#ifdef YAW_DES_INIT
		    desired_yaw = yaw;
			desired_yaw_init = yaw;
		#endif
  #endif
}


void update_correction_parameters()
{
  #ifdef IMU_ENABLE
    #ifdef TPA_ENABLE
	float PID_TPA_Coef = TPA_coefficient();
	Kp[0] = PID_P_roll*PID_TPA_Coef; Kp[1] = PID_P_pitch*PID_TPA_Coef; Kp[2] = PID_P_yaw*PID_TPA_Coef;       
	Ki[0] = PID_I_roll*PID_TPA_Coef; Ki[1] = PID_I_pitch*PID_TPA_Coef; Ki[2] = PID_I_yaw*PID_TPA_Coef;           
	Kd[0] = PID_D_roll*PID_TPA_Coef; Kd[1] = PID_D_pitch*PID_TPA_Coef; Kd[2] = PID_D_yaw*PID_TPA_Coef;  
	#endif
  
    error_roll = desired_roll - roll;
    error_pitch = desired_pitch - pitch;
	
	error_yaw = desired_yaw - yaw;
	error_alt = desired_alt - filtHz;
	error_alt_speed = 0 - filtVz;
    int16_t error2_roll;
    int16_t error2_pitch;
    
    corrI_roll += Ki[0]*0.001*samplingTimeCntl*error_roll;
    corrI_pitch += Ki[1]*0.001*samplingTimeCntl*error_pitch;
	corrI_yaw += Ki[2]*0.001*samplingTimeCntl*error_yaw;
	corrI_alt += Ki[3]*0.001*samplingTimeCntl*error_alt;
	corrI_alt_speed += Ki[4]*0.001*samplingTimeCntl*error_alt_speed;
	
    corr_roll = Kp[0]*error_roll + corrI_roll + Kd[0]*(error_roll - errorPrev_roll)/(samplingTimeCntl*0.001);
    corr_pitch = Kp[1]*error_pitch + corrI_pitch + Kd[1]*(error_pitch - errorPrev_pitch)/(samplingTimeCntl*0.001);
	corr_yaw = Kp[2]*error_yaw + corrI_yaw + Kd[2]*(error_yaw - errorPrev_yaw)/(samplingTimeCntl*0.001);
	corr_alt = Kp[3]*error_alt + corrI_alt + Kd[3]*(error_alt - errorPrev_alt)/(samplingTimeCntl*0.001);
	corr_alt_speed = Kp[4]*error_alt_speed + corrI_alt_speed + Kd[4]*(error_alt_speed - errorPrev_alt_speed)/(samplingTimeCntl*0.001);
	
	#ifdef PID_yaw_SAT
	corr_yaw = min(max(corr_yaw, -PID_yaw_SAT), PID_yaw_SAT);
	#endif
	
	#if PIDAltLevel == 2
		error2_alt = corr_alt - filtVz; 
		corr2I_alt += K2i[3]*0.001*samplingTimeCntl*error2_alt;
		corr2_alt = K2p[3]*error2_alt + corr2I_alt + K2d[3]*(error2_alt - error2Prev_alt)/(samplingTimeCntl*0.001);
		error2Prev_alt = error2_alt;
		corr_alt_total = corr2_alt;
	#else
		corr_alt_total = corr_alt + corr_alt_speed;
	#endif
	
	#ifdef TAKEOFF_AUTO
	if(state==5)
	{
		corr_alt_total = 0;
	}
	#endif
	#ifdef PID_alt_SAT
	corr_alt_total = min(max(corr_alt_total, -PID_alt_SAT), PID_alt_SAT);
	#endif
	
    errorPrev_roll = error_roll;
    errorPrev_pitch = error_pitch;
	errorPrev_yaw = error_yaw;
	errorPrev_alt = error_alt;
	errorPrev_alt_speed = error_alt_speed;
    
	droll = (roll-roll_prev)/(samplingTimeCntl*0.001);
	dpitch = (pitch-pitch_prev)/(samplingTimeCntl*0.001);

    error2_roll = corr_roll - droll; 
    error2_pitch = corr_pitch - dpitch;
    corr2I_roll += K2i[0]*0.001*samplingTimeCntl*error2_roll;
    corr2I_pitch += K2i[1]*0.001*samplingTimeCntl*error2_pitch;
    corr2_roll = K2p[0]*error2_roll + corr2I_roll + K2d[0]*(error2_roll - error2Prev_roll)/(samplingTimeCntl*0.001);
    corr2_pitch = K2p[1]*error2_pitch + corr2I_pitch + K2d[1]*(error2_pitch - error2Prev_pitch)/(samplingTimeCntl*0.001);
    error2Prev_roll = error2_roll;
    error2Prev_pitch = error2_pitch;
    
    roll_prev = roll;
    pitch_prev = pitch;
    
  if(state < 4)
  {
	corrI_roll = 0;
    corrI_pitch = 0;
	corrI_yaw = 0;
    corr_roll = 0;
    corr_pitch = 0;
	corr2I_roll = 0;
    corr2I_pitch = 0;
    corr2_roll = 0;
    corr2_pitch = 0;
	corr_yaw = 0;
	corr_alt = 0;
	corr_alt_speed = 0;
	corr_alt_total = 0;
  }
    
  #else
    corrI_roll = 0;
    corrI_pitch = 0;
	corrI_yaw = 0;
    corr_roll = 0;
    corr_pitch = 0;
	corr_yaw = 0
	corr_alt_speed = 0;
	corr_alt = 0;
	corr_alt_total = 0;
  #endif
}

void update_motors_control()
{
  analogWrite(MOTOR_1_PIN, mc1);
  analogWrite(MOTOR_2_PIN, mc2);
  analogWrite(MOTOR_3_PIN, mc3);
  analogWrite(MOTOR_4_PIN, mc4);
}

void update_control_mixing()
{
  #ifdef IMU_ENABLE
    #if PIDLevel == 2
		#ifdef PID_ATTITUDE_SAT
			corr2_roll = min(max(corr2_roll, -PID_ATTITUDE_SAT), PID_ATTITUDE_SAT);
			corr2_pitch = min(max(corr2_pitch, -PID_ATTITUDE_SAT), PID_ATTITUDE_SAT);
		#endif
		
        #ifdef INVERT_CMD
        mc1 = (int16_t)(-corr2_pitch -corr_yaw + corr_alt_total);
        mc4 = (int16_t)(corr2_pitch -corr_yaw + corr_alt_total);
        mc2 = (int16_t)(corr2_roll + corr_yaw + corr_alt_total);
        mc3 = (int16_t)(-corr2_roll + corr_yaw + corr_alt_total);
        #else
        mc1 = (int16_t)(corr2_pitch -corr_yaw + corr_alt_total);
        mc4 = (int16_t)(-corr2_pitch -corr_yaw + corr_alt_total);
        mc2 = (int16_t)(corr2_roll + corr_yaw + corr_alt_total);
        mc3 = (int16_t)(-corr2_roll + corr_yaw + corr_alt_total);
        #endif
    #else
        #ifdef INVERT_CMD
        mc1 = (int16_t)(-corr_pitch -corr_yaw + corr_alt_total);
        mc4 = (int16_t)(corr_pitch -corr_yaw + corr_alt_total);
        mc2 = (int16_t)(corr_roll + corr_yaw + corr_alt_total);
        mc3 = (int16_t)(-corr_roll + corr_yaw + corr_alt_total);
        #else
        mc1 = (int16_t)(corr_pitch -corr_yaw + corr_alt_total);
        mc4 = (int16_t)(-corr_pitch -corr_yaw + corr_alt_total);
        mc2 = (int16_t)(corr_roll + corr_yaw + corr_alt_total);
        mc3 = (int16_t)(-corr_roll + corr_yaw + corr_alt_total);
        #endif

    #endif
    /* Adding the throttle */ 
  if(ALT_AUTO == 0)
  {
	staticThrottle = throttle;
  }
  else
  {
    if(state == 4)
    {
		staticThrottle=flightThrottle;
    }
	if(state == 5)
    {
		staticThrottle=takeOffThrottle;
    }
  }
  
  if(state<4)
  {
	  staticThrottle=0;
  }
  
  mc1 += staticThrottle;
  mc2 += staticThrottle;
  mc3 += staticThrottle;
  mc4 += staticThrottle;

  mc1 = throttleSaturation(mc1);
  mc2 = throttleSaturation(mc2);
  mc3 = throttleSaturation(mc3);
  mc4 = throttleSaturation(mc4);
  #ifdef MOTORSATURATIONDETECTION
  if(mc1 == 255)
  {
      saturationMotor[0] = 1;
  }
  if(mc2 == 255)
  {
      saturationMotor[1] = 1;
  }
  if(mc3 == 255)
  {
      saturationMotor[2] = 1;
  }
  if(mc4 == 255)
  {
      saturationMotor[3] = 1;
  }
  #endif
  #else
  mc1 = throttle;
  mc2 = throttle;
  mc3 = throttle;
  mc4 = throttle;
  #endif
}

#ifdef BATT_SOC
void Batt_Read()
{
	battVoltage = analogRead(BATT_PIN)*2*3.3/4095;
	battOCV = battVoltage;
	battSOC = Batt_OCV2SOC(battOCV);
}

float Batt_OCV2SOC(float voltage_in)
{
	int i;
	float soc;
	float voltage;
	voltage=voltage_in+OCV_offset;
	
	for(i=1;i<=OCV_size;i++)
	{
		if(voltage <= OCV_V[i] && voltage > OCV_V[i-1])
		{
			soc = OCV_S[i-1] + (OCV_S[i]-OCV_S[i-1])*(voltage-OCV_V[i-1])/(OCV_V[i]-OCV_V[i-1]);
		}
	}
	return soc;
}

void coulombCount()
{
	battCoulombCounterSOC += Thrust2Amp*CoulombCountTs*0.001*(mc1 + mc2 + mc3 + mc4)/(255*battCapacity);
}

void updateDynamicTOTimeOut(float soc)
{
	int i;
	for(i=1;i<=OCV_size;i++)
	{
		if(soc <= OCV_S[i] && soc > OCV_S[i-1])
		{
			timeOutTO = OCV_TOTiOut[i-1] + (OCV_TOTiOut[i]-OCV_TOTiOut[i-1])*(soc-OCV_S[i-1])/(OCV_S[i]-OCV_S[i-1]);
		}
	}
}

void updateDynamicThrottles(float soc)
{
	int i;
	for(i=1;i<=OCV_size;i++)
	{
		if(soc <= OCV_S[i] && soc > OCV_S[i-1])
		{
			takeOffThrottle = OCV_ThrTo[i-1] + (OCV_ThrTo[i]-OCV_ThrTo[i-1])*(soc-OCV_S[i-1])/(OCV_S[i]-OCV_S[i-1]);
			flightThrottle = OCV_ThrFl[i-1] + (OCV_ThrFl[i]-OCV_ThrFl[i-1])*(soc-OCV_S[i-1])/(OCV_S[i]-OCV_S[i-1]);	
		}
	}
}

#endif

////////////////// COMMUNICATION SECTION //////////////////////////

void PPM_Read()
{
#ifdef PPM_RECV
  currentTimePPM_temp = micros(); 
  if((currentTimePPM_temp - currentTimePPM) > PPM_TSYNC*1000)
  {
      currentTimePPM = micros();
      PPM_state = 1;                                                        //Start of sequence read i.e 1st rising edge , first signal in the frame
      
  }
  else
  {
  if(PPM_state == 10)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[9] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 9)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[8] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 8)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[7] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 7)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[6] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 6)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[5] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 5)                                                            //Rising edge (6th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[4] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 4)                                                            //Rising edge (5th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[3] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 3)                                                            //Rising edge (4th), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[2] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 2)                                                            //Rising edge (3rd), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[1] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  if(PPM_state == 1)                                                            //Rising edge (2nd), marking the end of the 1st signal and the start of the 2nd signal.
  {
      PPM_CHN_DATA[0] = currentTimePPM_temp - currentTimePPM;
      currentTimePPM = micros();
      PPM_state++;
  }
  }
  
  PPM_Interprete();
  
#endif
}

void PPM_Interprete()
{
	#ifdef PPM_RECV
	desired_roll = 180*(PPM_CHN_DATA[0] - (PPM_HI+PPM_LO)/2)/(PPM_HI - PPM_LO); //Shall be between [-0.5;0.5] or [-90;90]
	desired_pitch = 180*(PPM_CHN_DATA[1] - (PPM_HI+PPM_LO)/2)/(PPM_HI - PPM_LO);
	
	throttle = (int16_t)(255*(PPM_CHN_DATA[2] - PPM_LO)/(PPM_HI - PPM_LO));
	if(throttle < 0){throttle=0;}
	if(throttle>=250 && PPM_init_ok==0)
	{
		PPM_init_ok=1;
	}
	if(throttle<TAKEOFF_THR_LO && PPM_init_ok==1)
	{
		PPM_init_ok=2;
	}
	if(PPM_init_ok != 2)
	{
		throttle=0;
	}
	else
	{
		if(throttle <= TAKEOFF_THR_LO)
		{
			state_prev=state;
			state = 3;
		}
	    
		if(state==3 && throttle>TAKEOFF_THR_LO)
		{
			state_prev=state;
			state = 4;
			LedStateON();
		}
		#ifdef TAKEOFF_AUTO
		if(state_prev==3 && state==4)
		{
			state_prev=state;
			state=5;
			currentTimeTO=System.ticks();
		}
		#endif
	}
	
	float desired_dyaw = 180*(PPM_CHN_DATA[3] - (PPM_HI+PPM_LO)/2)/(PPM_HI - PPM_LO);
	
	#ifdef PPM_ATTITUDE_MAX
	desired_roll = desired_roll*PPM_ATTITUDE_MAX/90;
	desired_pitch = desired_pitch*PPM_ATTITUDE_MAX/90;
	desired_dyaw = desired_dyaw*PPM_ATTITUDE_MAX/90;
	#endif
	
	desired_yaw = desired_yaw_init + desired_dyaw;
	
	#endif
}


void get_desired_param_1B()
{
  byte temp;
  byte command;
  byte value;
  
  //Test with AUTOMATIC mode ON: you should be able to loop here, to see in the console that the client connects and disconnects...etc.
  if(client.connected() && client_connected == 0)
  {
      Particle.publish("TCP","Client connected.");
      client_connected = 1;
  }
  
  if(client_connected)
  {
      #ifdef HEART_BEAT
      if(heart_beat_count > HEART_BEAT_THR)
      {
        Particle.publish("TCP","Client disconnected.");
        client_connected = 0;
	    client.stop();
	    server.stop();
	    server.begin();
	    heart_beat_count = 0;
      }
      else
      {
        heart_beat_count++;
        if(client.available())
        {
            temp = (byte)client.read();
            command = temp >> 5;
            value = temp & 31;
            get_desired_param_1BConv(command, value);
        }
      }
      #else
      if(client.available())
      {
        temp = (byte)client.read();
        command = temp >> 5;
        value = temp & 31;
        get_desired_param_1BConv(command, value);
      }
      #endif
  }
  else
  {
      client = server.available();
  }
}

void get_desired_param_1BConv(byte command, byte value)
{
  String commandConv;
  
  switch (command) {
      case 0:                                               //STOP                       
        standby();
      break;
      case 1:                                               //Throttle
        throttle = ((uint8_t)value)*FastComm1B_THROTTLE;
	    if(throttle <= 0)
	    {
		    state = 3;
	    }
	    if(state==3 && throttle>0)
	    {
		    state = 4;
		    LedStateON();
	    }
		
      break;
      case 2:                                               //Pitch positive
        desired_pitch = ((float)value)*FastComm1B_ANGLE; 
        Particle.publish("desired_pitch+",String(desired_pitch));
      break;
      case 3:                                               //Pitch negative
        desired_pitch = -1*((float)value)*FastComm1B_ANGLE;              
        Particle.publish("desired_pitch-",String(desired_pitch));
      break;
      case 4:                                               //Roll positive
        desired_roll = ((float)value)*FastComm1B_ANGLE;
        Particle.publish("desired_roll+",String(desired_roll));
      break;
      case 5:                                               //Roll negative
        desired_roll = -1*((float)value)*FastComm1B_ANGLE; 
        Particle.publish("desired_roll-",String(desired_roll));
      break;
      case 6:                                               //Stabilize
        desired_roll = 0;
        desired_pitch = 0;
        Particle.publish("Stabilized",String(1));
      break;
      case 7:
        if((uint8_t)value == 0)
        {
            System.reset();
        }
        if((uint8_t)value == 1)
        {
            //Heart beat
            #ifdef HEART_BEAT
            heart_beat_count=0;
            #endif
        }
        if((uint8_t)value == 2)
        {
            //Connect to the cloud
            Particle.connect();
        }
        if((uint8_t)value == 3)
        {
            //Disconnect from the cloud
            Particle.disconnect();
        }
      break;
    default: 
      //Do nothing
    break;
  }
}


void wifi_init(String eSSID, String PASS, int tcp_port)
{
  server = TCPServer(tcp_port);
  server.begin();
  Particle.publish("TCP","Server ON.");
}

 ///////////////////// MOTOR CONTROL SECTION //////////////////////
void testMotorsPower(uint8_t thrust)
{
  uint8_t tmp = throttleSaturation(thrust);
  analogWrite(MOTOR_1_PIN, (int)tmp);
  analogWrite(MOTOR_2_PIN, (int)tmp);
  analogWrite(MOTOR_3_PIN, (int)tmp);
  analogWrite(MOTOR_4_PIN, (int)tmp);
}

void testMotor(int motor)
{
    if(motor == 1)
    {
        analogWrite(MOTOR_1_PIN, 200);
    }
    if(motor == 2)
    {
        analogWrite(MOTOR_2_PIN, 200);
    }
    if(motor == 3)
    {
        analogWrite(MOTOR_3_PIN, 200);
    }
    if(motor == 4)
    {
        analogWrite(MOTOR_4_PIN, 200);
    }
}


uint8_t throttleSaturation(uint8_t thrust_unSat)
{
  if(thrust_unSat > 255)
  {
    return 255;
  }
  else if(thrust_unSat < 0)
  {
    return 0;
  }
  else
  {
    return thrust_unSat;
  }
}

void motors_init()
{
  pinMode(MOTOR_1_PIN, OUTPUT);
  analogWrite(MOTOR_1_PIN, 0);
  pinMode(MOTOR_2_PIN, OUTPUT);
  analogWrite(MOTOR_2_PIN, 0);
  pinMode(MOTOR_3_PIN, OUTPUT);
  analogWrite(MOTOR_3_PIN, 0);
  pinMode(MOTOR_4_PIN, OUTPUT);
  analogWrite(MOTOR_4_PIN, 0);
}

void motors_stop()
{
  mc1 = 0;
  mc2 = 0;
  mc3 = 0;
  mc4 = 0;
  analogWrite(MOTOR_1_PIN, 0);
  analogWrite(MOTOR_2_PIN, 0);
  analogWrite(MOTOR_3_PIN, 0);
  analogWrite(MOTOR_4_PIN, 0);
}


///////////////////// DEBUG SECTION /////////////////////



void send_tcp(String data)
{
    const unsigned char * dataToSend = (const unsigned char *)data.c_str();
    server.write(dataToSend, strlen((char*)dataToSend));
}

void transmit_flightParam()
{
  String data = "roll ";
  data += (String)roll; data += " pitch ";
  data += (String)pitch; data += " yaw ";
  data += (String)yaw;
  data += returnCariage;
  send_tcp(data);
}

void publish_flightParam()
{
  String data = "roll=";
  data += (String)roll; data += ", pitch=";
  data += (String)pitch; data += ", yaw=";
  data += (String)yaw; 
  Particle.publish("FLIGHT PARAM", data);
}

void publish_timings()
{
    String data = "CTL_TimMax=";
    data += String(CTL_timing_MAX);
    data += ",CTL_TimMin=";
    data += String(CTL_timing_MIN);
    Particle.publish("STATS",data);
}

#ifdef BATT_SOC
void publish_battStatus()
{
	Batt_Read();
    String socDisplay = "SoC (Ocv) [%]: " + String(battSOC) + " - OCV [V]: " + String(battOCV);
	Particle.publish("SYS", socDisplay);
}
#endif
  
void publish_altimeter()
{
    String data = "HzBar:";
    data += String(HzBar);
    data += " cm, filtHz: ";
    data += String(filtHz);
    data += " cm";
    Particle.publish("STATS", data);
}

/////////////// DATA LOGGING /////////////

#ifdef QualityFactor
void updateQf()
{
	QfTime+=samplingTimeCntl*0.001;
	if(abs(roll-desired_roll) <= QfGood)
	{
		QfCntrRoll++;
	}
	if(abs(roll-desired_roll) > QfBad)
	{
		QfCntrRoll--;
	}
	if(abs(pitch-desired_pitch) <= QfGood)
	{
		QfCntrPitch++;
	}
	if(abs(pitch-desired_pitch) > QfBad)
	{
		QfCntrPitch--;
	}
}
#endif

void updateLog()
{
	#ifdef dataLoggingBasicAnalysis
	buffer.stampTime[buffer.idx] = millis();  
	buffer.pitchLog[buffer.idx] = pitch;
	buffer.rollLog[buffer.idx] = roll;
	buffer.throttleLog[buffer.idx] = throttle;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
	#endif
	#ifdef dataLoggingPPMAnalysis
	buffer.stampTime[buffer.idx] = millis();  
	buffer.pitchLog[buffer.idx] = pitch;
	buffer.rollLog[buffer.idx] = roll;
	buffer.yawLog[buffer.idx] = yaw;
	buffer.throttleLog[buffer.idx] = throttle;
	buffer.mc1Log[buffer.idx] = mc1;
	buffer.mc2Log[buffer.idx] = mc2;
	buffer.mc3Log[buffer.idx] = mc3;
	buffer.mc4Log[buffer.idx] = mc4;
	buffer.altitudeLog[buffer.idx] = altitude;
	buffer.desired_rollLog[buffer.idx] = desired_roll;
	buffer.desired_pitchLog[buffer.idx] = desired_pitch;
	buffer.desired_yawLog[buffer.idx] = desired_yaw;
	buffer.stateLog[buffer.idx] = state;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
	#endif
    #ifdef dataLoggingIMUAnalysis
    buffer.stampTime[buffer.idx] = millis();
    buffer.pitchLog[buffer.idx] = pitch;
    buffer.rollLog[buffer.idx] = roll;
    buffer.filtPitchAccLog[buffer.idx] = pitchAcc;
    buffer.filtRollAccLog[buffer.idx] = rollAcc;
    buffer.throttleLog[buffer.idx] = throttle;
    buffer.filtRollGyroRateLog[buffer.idx] = rollRateGyro;
    buffer.filtPitchGyroRateLog[buffer.idx] = pitchRateGyro;
    buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
    #endif
	
	#ifdef dataLoggingAttitudeAnalysis
    buffer.stampTime[buffer.idx] = millis();
    buffer.pitchLog[buffer.idx] = pitch;
    buffer.rollLog[buffer.idx] = roll;
    buffer.throttleLog[buffer.idx] = throttle;
    buffer.mc1Log[buffer.idx] = mc1;
	buffer.mc2Log[buffer.idx] = mc2;
	buffer.mc3Log[buffer.idx] = mc3;
	buffer.mc4Log[buffer.idx] = mc4;
	buffer.error_rollLog[buffer.idx] = error_roll;
	buffer.error_pitchLog[buffer.idx] = error_pitch;
	buffer.corr_rollLog[buffer.idx] = corr_roll;
	buffer.corr_pitchLog[buffer.idx] = corr_pitch;
	buffer.drollLog[buffer.idx] = droll;
	buffer.dpitchLog[buffer.idx] = dpitch;
	buffer.corr2_rollLog[buffer.idx] = corr2_roll;
	buffer.corr2_pitchLog[buffer.idx] = corr2_pitch;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
    #endif
    #ifdef dataLoggingControlAnalysis
    buffer.stampTime[buffer.idx] = millis();
    buffer.pitchLog[buffer.idx] = pitch;
    buffer.rollLog[buffer.idx] = roll;
    buffer.throttleLog[buffer.idx] = throttle;
    buffer.mc1Log[buffer.idx] = mc1;
	buffer.mc2Log[buffer.idx] = mc2;
	buffer.mc3Log[buffer.idx] = mc3;
	buffer.mc4Log[buffer.idx] = mc4;
	buffer.error_rollLog[buffer.idx] = error_roll;
	buffer.error_pitchLog[buffer.idx] = error_pitch;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
    #endif
	#ifdef dataLoggingMAGAnalysis
    buffer.stampTime[buffer.idx] = millis();
    buffer.yawLog[buffer.idx] = yaw;
    buffer.throttleLog[buffer.idx] = battOCV;
	buffer.compassErrorLog[buffer.idx] = compassError;
	buffer.filtYawGyroLog[buffer.idx] = filtYawGyro;
	buffer.filtYawRateGyroLog[buffer.idx] = yawRateGyro;
	buffer.filtYawCompassLog[buffer.idx] = filtYawCompass;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
    #endif
	
	#ifdef dataLoggingPOSAnalysis
    buffer.stampTime[buffer.idx] = millis();
	buffer.rollLog[buffer.idx] = roll;
	buffer.pitchLog[buffer.idx] = pitch;
	#ifdef POS_EST
	buffer.filtHxLog[buffer.idx] = filtHx;
	buffer.filtHyLog[buffer.idx] = filtHy;
	buffer.pos_xLog[buffer.idx] = pos_x;
	buffer.pos_yLog[buffer.idx] = pos_y;
	buffer.AxLog[buffer.idx] = Ax;
	buffer.AyLog[buffer.idx] = Ay;
	buffer.filtXLog[buffer.idx] = filtX;
	buffer.filtYLog[buffer.idx] = filtY;
	#endif
	buffer.filtHzLog[buffer.idx] = filtHz;
	buffer.mc1Log[buffer.idx] = mc1;
	buffer.mc2Log[buffer.idx] = mc2;
	buffer.mc3Log[buffer.idx] = mc3;
	buffer.mc4Log[buffer.idx] = mc4;
	buffer.HzBarLog[buffer.idx] = HzBar;
	buffer.staticThrottleLog[buffer.idx] = staticThrottle;
	buffer.corr_alt_totalLog[buffer.idx] = corr_alt_total;
	buffer.idx++;
    if(buffer.idx == dataLoggingBufferSize)
    {
        buffer.idx = 0;
    }
    #endif

}


void transmit_DataLogChunk()
{
    String tmp = "";
	#ifdef dataLoggingBasicAnalysis
	int i;
	tmp = "Time, roll, pitch, throttle";
    send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
	    tmp += String(buffer.rollLog[i], 2) + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.throttleLog[i]);
		send_tcp(tmp);
		client.flush();
	}
	#endif
	#ifdef dataLoggingPPMAnalysis
	int i;
	tmp = "Time, roll, pitch, yaw, throttle, mc1, mc2, mc3, mc4, altitude, desired_roll, desired_pitch, desired_yaw, state";
    send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
	    tmp += String(buffer.rollLog[i], 2) + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.yawLog[i], 2) + ",";
		send_tcp(tmp);
		tmp = String(buffer.throttleLog[i]) + ",";
		tmp += String(buffer.mc1Log[i]) + ",";
		tmp += String(buffer.mc2Log[i]) + ",";
		tmp += String(buffer.mc3Log[i]) + ",";
		tmp += String(buffer.mc4Log[i]) + ",";
		send_tcp(tmp);
		tmp = String(buffer.altitudeLog[i], 2) + ",";
		tmp += String(buffer.desired_rollLog[i], 2) + ",";
		tmp += String(buffer.desired_pitchLog[i], 2) + ",";
		tmp += String(buffer.desired_yawLog[i], 2) + ",";
		tmp += String(buffer.stateLog[i], 2); 
		send_tcp(tmp);
		client.flush();
	}
	#endif
    #ifdef dataLoggingIMUAnalysis
    int i;
	tmp = "Time,roll,pitch,roll_acc,pitch_acc,roll_gyro_rate,pitch_gyro_rate,throttle";
    send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
	    tmp += String(buffer.rollLog[i], 2) + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.filtRollAccLog[i], 2) + ",";
		tmp += String(buffer.filtPitchAccLog[i], 2) + ",";
		tmp += String(buffer.filtRollGyroRateLog[i], 2) + ",";
		tmp += String(buffer.filtPitchGyroRateLog[i], 2) + ",";
		tmp += String(buffer.throttleLog[i]); 
		send_tcp(tmp);
		client.flush();
	}
    #endif

	#ifdef dataLoggingAttitudeAnalysis
    int i;
	tmp = "Time,roll,pitch,throttle,mc1,mc2,mc3,mc4,errorRoll,errorPitch,corrRoll,corrPitch,droll,dpitch,corr2Roll,corr2Pitch";
	send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
	    tmp += String(buffer.rollLog[i], 2) + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.throttleLog[i]) + ","; 
		tmp += String(buffer.mc1Log[i]) + ","; 
		tmp += String(buffer.mc2Log[i]) + ","; 
		tmp += String(buffer.mc3Log[i]) + ",";
		tmp += String(buffer.mc4Log[i]) + ",";
		send_tcp(tmp);
		tmp = String(buffer.error_rollLog[i], 2) + ","; 
		tmp += String(buffer.error_pitchLog[i], 2) + ","; 
		tmp += String(buffer.corr_rollLog[i], 2) + ","; 
		tmp += String(buffer.corr_pitchLog[i], 2) + ","; 
		tmp += String(buffer.drollLog[i], 2) + ","; 
		tmp += String(buffer.dpitchLog[i], 2) + ","; 
		tmp += String(buffer.corr2_rollLog[i], 2) + ","; 
		tmp += String(buffer.corr2_pitchLog[i], 2); 
		send_tcp(tmp);
		client.flush();
	}
    #endif
    #ifdef dataLoggingControlAnalysis
    int i;
	tmp = "Time,roll,pitch,throttle,mc1,mc2,mc3,mc4,errorRoll, errorPitch";
	send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
	    tmp += String(buffer.rollLog[i], 2) + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.throttleLog[i]) + ","; 
		tmp += String(buffer.mc1Log[i]) + ","; 
		tmp += String(buffer.mc2Log[i]) + ","; 
		send_tcp(tmp);
		tmp = String(buffer.mc3Log[i]) + ",";
		tmp += String(buffer.mc4Log[i]) + ",";
		tmp += String(buffer.error_rollLog[i], 2) + ","; 
		tmp += String(buffer.error_pitchLog[i], 2); 
		send_tcp(tmp);
		client.flush();
	}
    #endif
	#ifdef dataLoggingMAGAnalysis
    int i;
	tmp = "Time, yaw, throttle, yaw_gyro, yaw_gyro_rate, filtYawCompass, compassError";
    send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    tmp = "\n" + String(buffer.stampTime[i]) + ",";
		tmp += String(buffer.yawLog[i], 2) + ",";
		tmp += String(buffer.throttleLog[i])  + ",";
		tmp += String(buffer.filtYawGyroLog[i], 2) + ",";
		tmp += String(buffer.filtYawRateGyroLog[i], 2) + ",";
		tmp += String(buffer.filtYawCompassLog[i], 2) + ",";
		tmp += String(buffer.compassErrorLog[i], 2);
		send_tcp(tmp);
		client.flush();
	}
    #endif
	#ifdef dataLoggingPOSAnalysis
    int i;
	#ifdef POS_EST
	tmp = "Time, roll, pitch, filtHx, filtHy, filtHz, pos_x, pos_y, Ax, Ay, Az, filtX, filtY, filtZ, HzBar, staticThrottle, corr_alt_total";
	#else
	tmp = "Time, roll, pitch, filtHz, mc1, mc2, mc3, mc4, HzBar, staticThrottle, corr_alt_total";
	#endif
    send_tcp(tmp);
	for(i=0;i<dataLoggingBufferSize;i++)
	{
	    
		#ifdef POS_EST
		tmp = "\n" + String(buffer.stampTime[i]) + ",";
		tmp += String(buffer.rollLog[i], 2)  + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.filtHxLog[i], 2) + ",";
		tmp += String(buffer.filtHyLog[i], 2) + ",";
		tmp += String(buffer.filtHzLog[i], 2) + ",";
		send_tcp(tmp);
		tmp = String(buffer.pos_xLog[i], 2) + ",";
		tmp += String(buffer.pos_yLog[i], 2) + ",";
		tmp += String(buffer.AxLog[i], 2) + ",";
		tmp += String(buffer.AyLog[i], 2) + ",";
		tmp += String(buffer.AzLog[i], 2) + ",";
		send_tcp(tmp);
		tmp = String(buffer.filtXLog[i], 2) + ",";
		tmp += String(buffer.filtYLog[i], 2) + ",";
		tmp += String(buffer.filtZLog[i], 2) + ",";
		tmp += String(buffer.HzBarLog[i], 2) + ",";
		tmp += String(buffer.staticThrottleLog[i]) + ",";
		tmp += String(buffer.corr_alt_totalLog[i]);
		send_tcp(tmp);
		#else
		tmp = "\n" + String(buffer.stampTime[i]) + ",";
		tmp += String(buffer.rollLog[i], 2)  + ",";
		tmp += String(buffer.pitchLog[i], 2) + ",";
		tmp += String(buffer.filtHzLog[i], 2) + ",";
		tmp += String(buffer.mc1Log[i]) + ",";
		send_tcp(tmp);
		tmp = String(buffer.mc2Log[i]) + ",";
		tmp += String(buffer.mc3Log[i]) + ",";
		tmp += String(buffer.mc4Log[i]) + ",";
		tmp += String(buffer.HzBarLog[i], 2) + ",";
		tmp += String(buffer.staticThrottleLog[i]) + ",";
		tmp += String(buffer.corr_alt_totalLog[i]);
		send_tcp(tmp);
		#endif
		tmp = "";
		client.flush();
	}
    #endif
	
    delay(500);
    tmp = "*ENDOFFILE*";
    send_tcp(tmp);
    Particle.publish("LOG","Log transmitted.");
}



////////////////////////////// LED STATE ///////////////////////

void LedStateON()
{
    digitalWrite(LED_PIN, HIGH);   
}

void LedStateOFF()
{
    digitalWrite(LED_PIN, LOW);   
}
