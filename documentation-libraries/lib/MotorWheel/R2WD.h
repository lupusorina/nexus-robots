#include<MotorWheel.h>

#ifndef R2WD_H
#define R2WD_H

#define WHEELSPAN 290

class R2WD {
public:
	R2WD(MotorWheel* wheelLeft,MotorWheel* wheelRight,unsigned int wheelspanMM=WHEELSPAN);

	unsigned int getWheelspanMM() const;
	unsigned int setWheelspanMM(unsigned int wheelspan);

	// Car Direction Control
	unsigned char switchMotors();
	unsigned char switchMotorsReset();
	
	// Simple Actions
	unsigned int setCarStop();
	unsigned int setCarAdvance(unsigned int speedMMPS=0);
	unsigned int setCarBackoff(unsigned int speedMMPS=0);
	unsigned int setCarRotateLeft(unsigned int speedMMPS=0);
	unsigned int setCarRotateRight(unsigned int speedMMPS=0);

	unsigned int setCarUpperLeft(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarLowerLeft(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarLowerRight(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));
	unsigned int setCarUpperRight(unsigned int speedMMPS=0,unsigned int radiusMM=(WHEELSPAN>>1));

	// Staight Distance
	unsigned int setCarAdvanceDistance(unsigned int speedMMPS=0,unsigned long distance=0);
	unsigned int setCarBackoffDistance(unsigned int speedMMPS=0,unsigned long distance=0);

	// Rotate(Spin) Angle(degree or radian)
	unsigned int setCarRotateLeftAngle(unsigned int speedMMPS=0,float radian=0);
	unsigned int setCarRotateRightAngle(unsigned int speedMMPS=0,float radian=0);


	// ARC Path
	unsigned int setCarUpperLeftTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarLowerLeftTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarUpperRightTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarLowerRightTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned long duration=5000,unsigned int uptime=500);
 

	unsigned int setCarUpperLeftAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarLowerLeftAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarUpperRightAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500);
	unsigned int setCarLowerRightAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,float radian=0,unsigned int uptime=500); 

	// Single Wheel Control
	unsigned int wheelLeftSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelLeftGetSpeedMMPS() const;
	unsigned int wheelRightSetSpeedMMPS(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int wheelRightGetSpeedMMPS() const;

	// PID Control
	bool PIDEnable(float kc=KC,float taui=TAUI,float taud=TAUD,unsigned int interval=1000);
	bool PIDRegulate();
	void delayMS(unsigned int ms=100, bool debug=false);

	// Progressive Speed Control
	unsigned int getCarSpeedMMPS() const;
	unsigned int setCarSpeedMMPS(unsigned int speedMMPS=0,unsigned int ms=1000);
	unsigned int setCarSpeedMMPSArc(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,unsigned int ms=1000);
	//unsigned int setCarSpeedMMPSD(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0,unsigned int ms=1000);
	unsigned int setCarSlow2Stop(unsigned int ms=1000);

	// Demo Actions
	void demoActions(unsigned int speedMMPS=100,unsigned int duration=5000,unsigned int uptime=500,bool debug=false);
	// Debug
	void debugger(bool wheelLeftDebug=true,bool wheelRightDebug=true) const;

	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
	};
	unsigned char getCarStat() const;

	enum {
		MOTORS_FB,
		MOTORS_BF,
	};
	unsigned char getSwitchMotorsStat() const;
	
	unsigned int getRadiusMM() const;

private:
	MotorWheel* _wheelLeft;
	MotorWheel* _wheelRight;

	unsigned int _wheelspanMM;


	unsigned char _carStat;
	unsigned char setCarStat(unsigned char stat);

	unsigned char _switchMotorsStat;
	unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

	unsigned int _radiusMM;
	unsigned int setRadiusMM(unsigned int radiusMM);

	R2WD();	

	// Base Actions
	unsigned int setMotorAll(unsigned int speedMMPS=0,bool dir=DIR_ADVANCE);
	unsigned int setMotorAllStop();
	unsigned int setMotorAllAdvance(unsigned int speedMMPS=0);
	unsigned int setMotorAllBackoff(unsigned int speedMMPS=0);

	unsigned int setCarAdvanceBase(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0);
	unsigned int setCarBackoffBase(unsigned int speedMMPSL=0,unsigned int speedMMPSR=0);

	unsigned int setCarRotateAngle(unsigned int speedMMPS=0,float radian=0);

	unsigned int setCarStraightDistance(unsigned int speedMMPS=0,unsigned long distance=0);

	unsigned int setCarArcBase(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN);
	unsigned int setCarArcTime(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,
				unsigned long duration=5000,unsigned int uptime=500);
	unsigned int setCarArcAngle(unsigned int speedMMPS=0,unsigned int radiusMM=WHEELSPAN,
				float radian=0,unsigned int uptime=500);
};

#endif





