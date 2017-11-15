#include<R2WD.h>

R2WD::R2WD(MotorWheel* wheelLeft,MotorWheel* wheelRight,unsigned int wheelspanMM):
			_wheelLeft(wheelLeft),_wheelRight(wheelRight),_wheelspanMM(wheelspanMM) {
	setSwitchMotorsStat(MOTORS_FB);
}

unsigned int R2WD::getWheelspanMM() const {
	return _wheelspanMM;
}
unsigned int R2WD::setWheelspanMM(unsigned int wheelspanMM) {
	_wheelspanMM=wheelspanMM;
	return getWheelspanMM();
}

unsigned char R2WD::getSwitchMotorsStat() const {
	return _switchMotorsStat;
}
unsigned char R2WD::setSwitchMotorsStat(unsigned char switchMotorsStat) {
	if(MOTORS_FB<=switchMotorsStat && switchMotorsStat<=MOTORS_BF)
		_switchMotorsStat=switchMotorsStat;
	return getSwitchMotorsStat();
}
unsigned char R2WD::switchMotors() {
	if(getSwitchMotorsStat()==MOTORS_FB) setSwitchMotorsStat(MOTORS_BF);
	else setSwitchMotorsStat(MOTORS_FB);
	MotorWheel* temp=_wheelRight;
	_wheelRight=_wheelLeft;
	_wheelLeft=temp;
	return getSwitchMotorsStat();
}
unsigned char R2WD::switchMotorsReset() {
	if(getSwitchMotorsStat()==MOTORS_BF) switchMotors();
	return getSwitchMotorsStat();
}

unsigned char R2WD::getCarStat() const {
	return _carStat;
}
unsigned char R2WD::setCarStat(unsigned char carStat) {
    if(STAT_UNKNOWN<=carStat && carStat<=STAT_UPPERRIGHT)
        return _carStat=carStat;
    else
        return STAT_UNKNOWN;
}

unsigned int R2WD::getRadiusMM() const {
	switch(getCarStat()) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
			return 0; break;
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return getWheelspanMM()>>1; break;
	}
	return _radiusMM;
}
unsigned int R2WD::setRadiusMM(unsigned int radiusMM) {
	_radiusMM=radiusMM;
	return getRadiusMM();
}

unsigned int R2WD::wheelRightSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelRight->setSpeedMMPS(speedMMPS,dir);
}
unsigned int R2WD::wheelRightGetSpeedMMPS() const {
	return _wheelRight->getSpeedMMPS();
}
unsigned int R2WD::wheelLeftSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLeft->setSpeedMMPS(speedMMPS,dir);
}
unsigned int R2WD::wheelLeftGetSpeedMMPS() const {
	return _wheelLeft->getSpeedMMPS();
}


unsigned int R2WD::setMotorAll(unsigned int speedMMPS,bool dir) {
	wheelLeftSetSpeedMMPS(speedMMPS,dir);
	wheelRightSetSpeedMMPS(speedMMPS,dir);
	return wheelRightGetSpeedMMPS();
}
unsigned int R2WD::setMotorAllStop() {
	return setMotorAll(0,DIR_ADVANCE);
}
unsigned int R2WD::setMotorAllAdvance(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_ADVANCE);
}
unsigned int R2WD::setMotorAllBackoff(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_BACKOFF);
}
unsigned int R2WD::setCarStop() {
	setCarStat(STAT_STOP);
	return setMotorAll(0,DIR_ADVANCE);
}

unsigned int R2WD::setCarAdvanceBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPSR,DIR_BACKOFF);
	return getCarSpeedMMPS();
}
unsigned int R2WD::setCarBackoffBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL,DIR_BACKOFF);
	wheelRightSetSpeedMMPS(speedMMPSR,DIR_ADVANCE);
	return getCarSpeedMMPS();
}

unsigned int R2WD::setCarAdvance(unsigned int speedMMPS) {
	setCarStat(STAT_ADVANCE);
	return setCarAdvanceBase(speedMMPS,speedMMPS);
}
unsigned int R2WD::setCarBackoff(unsigned int speedMMPS) {
	setCarStat(STAT_BACKOFF);
	return setCarBackoffBase(speedMMPS,speedMMPS);
}

unsigned int R2WD::setCarRotateLeft(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATELEFT);
	return setMotorAllBackoff(speedMMPS);
}
unsigned int R2WD::setCarRotateRight(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATERIGHT);
	return setMotorAllAdvance(speedMMPS);
}

unsigned int R2WD::setCarArcBase(unsigned int speedMMPS,unsigned int radiusMM) {
	unsigned int delta=(int)((float)getWheelspanMM()/(radiusMM<<1)*speedMMPS);
	unsigned int V1=speedMMPS-delta;
	unsigned int V2=speedMMPS+delta;

	setRadiusMM(radiusMM);
	switch(getCarStat()) {
		case STAT_UPPERLEFT:
			setCarAdvanceBase(V1,V2); break;
		case STAT_LOWERLEFT:
			setCarBackoffBase(V1,V2); break;
		case STAT_UPPERRIGHT:
			setCarAdvanceBase(V2,V1); break;
		case STAT_LOWERRIGHT:
			setCarBackoffBase(V2,V1); break;
	}
	return getCarSpeedMMPS();
}


unsigned int R2WD::setCarUpperLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD::setCarLowerLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD::setCarLowerRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD::setCarUpperRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}


unsigned int R2WD::getCarSpeedMMPS() const {
	unsigned int speedMMPSL=wheelLeftGetSpeedMMPS();
	unsigned int speedMMPSR=wheelRightGetSpeedMMPS();
	return (speedMMPSL+speedMMPSR)>>1;
}
unsigned int R2WD::setCarSpeedMMPS(unsigned int speedMMPS,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (R2WD::*carAction)(unsigned int speedMMPS);
	switch(carStat) {
		case STAT_ADVANCE:
			carAction=&R2WD::setCarAdvance; break;
		case STAT_BACKOFF:
			carAction=&R2WD::setCarBackoff; break;
		case STAT_ROTATELEFT:
			carAction=&R2WD::setCarRotateLeft; break;
		case STAT_ROTATERIGHT:
			carAction=&R2WD::setCarRotateRight; break;
		case STAT_UPPERLEFT:
		case STAT_LOWERLEFT:
		case STAT_LOWERRIGHT:
		case STAT_UPPERRIGHT:
			return setCarSpeedMMPSArc(speedMMPS,getRadiusMM(),ms);break;
		default:
			return currSpeed; break;
	}

	if(ms<100 || abs(speedMMPS-currSpeed)<10) {
		return (this->*carAction)(speedMMPS);
	}

	for(int time=0,speed=currSpeed;time<=ms;time+=50) {
		speed=map(time,0,ms,currSpeed,speedMMPS);
		(this->*carAction)(speed);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}

unsigned int R2WD::setCarSpeedMMPSArc(unsigned int speedMMPS,unsigned int radiusMM,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (R2WD::*carAction)(unsigned int speedMMPS,unsigned int radiusMM);
	switch(carStat) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return setCarSpeedMMPS(speedMMPS,ms); break;
		case STAT_UPPERLEFT:
			carAction=&R2WD::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&R2WD::setCarLowerLeft; break;
		case STAT_LOWERRIGHT:
			carAction=&R2WD::setCarLowerRight; break;
		case STAT_UPPERRIGHT:
			carAction=&R2WD::setCarUpperRight; break;
		default:
			return currSpeed; break;
	}

	if(ms<100 || abs(speedMMPS-currSpeed)<10) {
		return (this->*carAction)(speedMMPS,radiusMM);
	}

	for(int time=0,speed=currSpeed;time<=ms;time+=50) {
		speed=map(time,0,ms,currSpeed,speedMMPS);
		(this->*carAction)(speed,radiusMM);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS,radiusMM);
	return getCarSpeedMMPS();
}

/*
unsigned int R2WD::setCarSpeedMMPSD(unsigned int speedMMPSL,unsigned int speedMMPSR,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeedL=wheelLeftGetSpeedMMPS();
	unsigned int currSpeedR=wheelRightGetSpeedMMPS();

	unsigned int (R2WD::*carActionD)(unsigned int speedMMPSL,unsigned int speedMMPSR);
	switch(carStat) {
		case STAT_UNKNOWN:	// no break here
		case STAT_STOP:
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
		case STAT_UPPERLEFT:
		case STAT_LOWERLEFT:
		case STAT_LOWERRIGHT:
		case STAT_UPPERRIGHT:
			return setCarSpeedMMPS(speedMMPSL,ms); break;
		case STAT_ADVANCED:
			carActionD=&R2WD::setCarAdvanceD; break;
		case STAT_BACKOFFD:
			carActionD=&R2WD::setCarBackoffD; break;
		default:
			return getCarSpeedMMPS(); break;
	}

	if(ms<100) {
		return (this->*carActionD)(speedMMPSL,speedMMPSR);
	}

	for(int time=0,speedL=currSpeedL,speedR=currSpeedR;time<=ms;time+=50) {
		speedL=map(time,0,ms,currSpeedL,speedMMPSL);
		speedR=map(time,0,ms,currSpeedR,speedMMPSR);
		(this->*carActionD)(speedL,speedR);
		delayMS(50);
	}

	(this->*carActionD)(speedMMPSL,speedMMPSR);
	return getCarSpeedMMPS();
}
 */

unsigned int R2WD::setCarSlow2Stop(unsigned int ms) {
	unsigned char carStat=getCarStat();
	//if(STAT_UNKNOWN<=STAT_UPPERLEFT || carStat<=STAT_LOWERRIGHT) {
	if(STAT_UPPERLEFT<=carStat && carStat<=STAT_UPPERRIGHT) {
		return setCarSpeedMMPSArc(0,getRadiusMM(),ms);
	}
	return setCarSpeedMMPS(0,ms);
}
 

bool R2WD::PIDEnable(float kc,float taui,float taud,unsigned int interval) {
	return _wheelLeft->PIDEnable(kc,taui,taud,interval) && 
			_wheelRight->PIDEnable(kc,taui,taud,interval);
}
bool R2WD::PIDRegulate() {
	return _wheelLeft->PIDRegulate() && _wheelRight->PIDRegulate();
}
/*
void R2WD::delayMS(unsigned long ms,bool debug) {
	for(int i=0;i<ms;i+=SAMPLETIME) {
		PIDRegulate();
		if(debug && (i%500==0)) debugger();
		delay(SAMPLETIME);
	}
}
 */
void R2WD::delayMS(unsigned int ms,bool debug) {
	for(unsigned long endTime=millis()+ms;millis()<endTime;) {
		PIDRegulate();
		if(debug && (millis()%500==0)) debugger();
		if(endTime-millis()>=SAMPLETIME) delay(SAMPLETIME);
		else delay(endTime-millis());
	}
}

void R2WD::demoActions(unsigned int speedMMPS,unsigned int duration,unsigned int uptime,bool debug) {
	unsigned int (R2WD::*carAction[])(unsigned int speedMMPS)={
		&R2WD::setCarAdvance,
		&R2WD::setCarBackoff,
		&R2WD::setCarRotateLeft,
		&R2WD::setCarRotateRight,
	};
	unsigned int (R2WD::*carAction2[])(unsigned int speedMMPS,unsigned int radiusMM)={
		&R2WD::setCarUpperLeft,
		&R2WD::setCarLowerLeft,
		&R2WD::setCarUpperRight,
		&R2WD::setCarLowerRight,
	};

	for(int i=0;i<8;++i) {
		if(i<4) {
		Serial.println(i,DEC);
			(this->*carAction[i])(0);
			setCarSpeedMMPS(speedMMPS,uptime);
		} else {
			(this->*carAction2[i-4])(0,500);
			setCarSpeedMMPSArc(speedMMPS,getRadiusMM(),uptime);
		}
		delayMS(duration,debug);
		setCarSlow2Stop(uptime);
	}
    //setCarStop();
    //delayMS(duration,debug);
    //switchMotors();
}


unsigned int R2WD::setCarArcTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	delayMS(duration);
	setCarSlow2Stop(uptime);
	return uptime<<1+duration;
}
unsigned int R2WD::setCarUpperLeftTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarUpperLeft(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD::setCarLowerLeftTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarLowerLeft(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD::setCarUpperRightTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarUpperRight(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD::setCarLowerRightTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarLowerRight(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
 
/*
unsigned int R2WD::setCarArcAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	if(radian<0) return 0;
	unsigned int (R2WD::*carAction)(unsigned int speedMMPS,unsigned int radiusMM);
	unsigned long timeMS=(long)(radiusMM*radian/speedMMPS*1000);
	switch(getCarStat()) {
		case STAT_UPPERLEFT:
			carAction=&R2WD::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&R2WD::setCarLowerLeft; break;
		case STAT_UPPERRIGHT:
			carAction=&R2WD::setCarUpperRight; break;
		case STAT_LOWERRIGHT:
			carAction=&R2WD::setCarLowerRight; break;
		default:
			return getCarSpeedMMPS(); break;
	}
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	//(this->*carAction)(speedMMPS,radiusMM);
	delayMS(timeMS-uptime);
	setCarSlow2Stop(uptime);
	return timeMS;
} 
 */
unsigned int R2WD::setCarArcAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	if(radian<0) return 0;
	unsigned long timeMS=(long)(radiusMM*radian/speedMMPS*1000);
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	delayMS(timeMS-uptime);
	setCarSlow2Stop(uptime);
	return timeMS;
}
unsigned int R2WD::setCarUpperLeftAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarUpperLeft(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}
unsigned int R2WD::setCarLowerLeftAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarLowerLeft(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}
unsigned int R2WD::setCarUpperRightAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarUpperRight(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian);
}
unsigned int R2WD::setCarLowerRightAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarLowerRight(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}


unsigned int R2WD::setCarRotateAngle(unsigned int speedMMPS,float radian) {
	if(radian<0) return 0;
	unsigned long timeMS=(long)((getWheelspanMM()>>1)*radian/speedMMPS*1000);
	delayMS(timeMS);
	return timeMS;
}
unsigned int R2WD::setCarRotateLeftAngle(unsigned int speedMMPS,float radian) {
	setCarRotateLeft(speedMMPS);
	return setCarRotateAngle(speedMMPS,radian);
}
unsigned int R2WD::setCarRotateRightAngle(unsigned int speedMMPS,float radian) {
	setCarRotateRight(speedMMPS);
	return setCarRotateAngle(speedMMPS,radian);
}

unsigned int R2WD::setCarStraightDistance(unsigned int speedMMPS,unsigned long distance) {
	unsigned long timeMS=distance*1000/speedMMPS;
	delayMS(timeMS);
	return timeMS;
}
unsigned int R2WD::setCarAdvanceDistance(unsigned int speedMMPS,unsigned long distance) {
	setCarAdvance(speedMMPS);
	return setCarStraightDistance(speedMMPS,distance);
}
unsigned int R2WD::setCarBackoffDistance(unsigned int speedMMPS,unsigned long distance) {
	setCarBackoff(speedMMPS);
	return setCarStraightDistance(speedMMPS,distance);
}

void R2WD::debugger(bool wheelLeftDebug,bool wheelRightDebug) const {
	if(wheelLeftDebug) _wheelLeft->debugger();
	if(wheelRightDebug) _wheelRight->debugger();
}






