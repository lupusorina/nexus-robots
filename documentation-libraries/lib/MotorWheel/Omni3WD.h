#include<MotorWheel.h>

/*
	wheelLeft	wheelRight
		//			\\



			 ==
		wheelBack
 */

#ifndef Omni3WD_H
#define Omni3WD_H

class Omni3WD {
public:
	Omni3WD(MotorWheel* wheelBack,MotorWheel* wheelRight,MotorWheel* wheelLeft);
	unsigned char switchMotorsLeft();
	unsigned char switchMotorsRight();
	unsigned char switchMotorsReset();
	
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);
	unsigned int setCarStop();
	unsigned int setCarAdvance(unsigned int speedMMPS=0);
	unsigned int setCarBackoff(unsigned int speedMMPS=0);
	unsigned int setCarLeft(unsigned int speedMMPS=0);
	unsigned int setCarRight(unsigned int speedMMPS=0);
	unsigned int setCarRotateLeft(unsigned int speedMMPS=0);
	unsigned int setCarRotateRight(unsigned int speedMMPS=0);

	unsigned int getCarSpeedMMPS() const;
	unsigned int setCarSpeedMMPS(unsigned int speedMMPS=0,unsigned int ms=1000);
	unsigned int setCarSlow2Stop(unsigned int ms=1000);

	unsigned int wheelBackSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	//unsigned int wheelBackGetSpeedMMPS() const;
	int wheelBackGetSpeedMMPS() const;
	unsigned int wheelRightSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	//unsigned int wheelRightGetSpeedMMPS() const;
	int wheelRightGetSpeedMMPS() const;
	unsigned int wheelLeftSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	//unsigned int wheelLeftGetSpeedMMPS() const;
	int wheelLeftGetSpeedMMPS() const;
	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int interval=1000);
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false);
	void demoActions(unsigned int speedMMPS=100,unsigned int duration=5000,
					unsigned int uptime=500,bool debug=false);
	void demoActions_Orginal(unsigned int speedMMPS = 20,unsigned int ms = 5000,bool debug = false) ;
	void debugger(bool wheelBackDebug=true,bool wheelRightDebug=true,bool wheelLeftDebug=true) const;

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_LEFT,
			STAT_RIGHT,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_BRL,
		MOTORS_LBR,
		MOTORS_RLB,
	};
	unsigned char getSwitchMotorsStat() const;
	
private:
	MotorWheel* _wheelBack;
	MotorWheel* _wheelRight;
	MotorWheel* _wheelLeft;

	unsigned char _carStat;
	unsigned char setCarStat(unsigned char stat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

	Omni3WD();	

};

#endif





