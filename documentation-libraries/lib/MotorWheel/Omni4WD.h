#include<MotorWheel.h>

#ifndef Omni4WD_H
#define Omni4WD_H


/*
	Mecanum4WD
			  Front MOTORS_FB
	wheelUL	\\		// wheelUR


	wheelLL	//		\\ wheelLR
			  Back MOTORS_BF
 */


/*
	Omni4WD
			  Front MOTORS_FB
	wheelUL	//		\\ wheelUR


	wheelLL	\\		// wheelLR
			  Back MOTORS_BF
 */
#ifndef WHEELSPAN
#define WHEELSPAN 300
#endif
class Omni4WD {
public:
	Omni4WD(MotorWheel* wheelUL,MotorWheel* wheelLL,
			MotorWheel* wheelLR,MotorWheel* wheelUR,unsigned int wheelspan=WHEELSPAN);
	unsigned char switchMotors();
	unsigned char switchMotorsReset();
	
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);
	unsigned int setCarStop(unsigned int ms=0);

	int setCarMove(int speedMMPS,float rad,float omega=0);
	int setCarAdvance(int speedMMPS=0);
	int setCarBackoff(int speedMMPS=0);
	int setCarLeft(int speedMMPS=0);
	int setCarRight(int speedMMPS=0);

	float setCarRotate(float omega);
	int setCarRotateLeft(int speedMMPS=0);
	int setCarRotateRight(int speedMMPS=0);

	int setCarUpperLeft(int speedMMPS=0);
	int setCarLowerLeft(int speedMMPS=0);
	int setCarUpperRight(int speedMMPS=0);
	int setCarLowerRight(int speedMMPS=0);

	float getCarSpeedRad() const;
	int getCarSpeedMMPS() const;
	int setCarSpeedMMPS(int speedMMPS=0,unsigned int ms=1000);
	int setCarSlow2Stop(unsigned int ms=1000);

	int wheelULGetSpeedMMPS() const;
	unsigned int wheelULSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelULSetSpeedMMPS(int speedMMPS);
	int wheelLLGetSpeedMMPS() const;
	unsigned int wheelLLSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelLLSetSpeedMMPS(int speedMMPS);
	int wheelURGetSpeedMMPS() const;
	unsigned int wheelURSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelURSetSpeedMMPS(int speedMMPS);
	int wheelLRGetSpeedMMPS() const;
	unsigned int wheelLRSetSpeedMMPS(unsigned int speedMMPS,bool dir);
	int wheelLRSetSpeedMMPS(int speedMMPS);

	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int interval=1000);
	bool PIDDisable();		// 201209
	bool PIDGetStatus();	// 201209
	float PIDGetP_Param();	// 201210
	float PIDGetI_Param();	// 201210
	float PIDGetD_Param();	// 201210
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false,unsigned char* actBreak = 0);
	void demoActions(unsigned int speedMMPS=100,unsigned int duration=5000,unsigned int uptime=500,bool debug=false);
	void debugger(bool wheelULDebug=true,bool wheelLLDebug=true,
					bool wheelLRDebug=true,bool wheelURDebug=true) const;

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_LEFT,
			STAT_RIGHT,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
			ACTIONTYPES=STAT_UPPERRIGHT,	// 201209
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_FB,	// FrontBack
		MOTORS_BF,	// BackFront
	};
	unsigned char getSwitchMotorsStat() const;
	unsigned int getWheelspan() const;
	
private:
	MotorWheel* _wheelUL;	// UpperLeft
	MotorWheel* _wheelLL;	// LowerLeft
	MotorWheel* _wheelLR;	// LowerRight
	MotorWheel* _wheelUR;	// UpperRight

	unsigned int _wheelspan;	// 201208

	unsigned char _carStat;
	unsigned char setCarStat(unsigned char carStat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

	Omni4WD();	

};

#endif





