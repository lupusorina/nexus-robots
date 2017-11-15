#include<Omni3WD.h>

Omni3WD::Omni3WD(MotorWheel* wheelBack,MotorWheel* wheelRight,MotorWheel* wheelLeft):
			_wheelBack(wheelBack),_wheelRight(wheelRight),_wheelLeft(wheelLeft) {
	setSwitchMotorsStat(MOTORS_BRL);
}

unsigned char Omni3WD::getSwitchMotorsStat() const {
	return _switchMotorsStat;
}
unsigned char Omni3WD::setSwitchMotorsStat(unsigned char switchMotorsStat) {
	if(MOTORS_BRL<=switchMotorsStat && switchMotorsStat<=MOTORS_RLB)
		_switchMotorsStat=switchMotorsStat;
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::switchMotorsLeft() {	// --> BRL --> LBR --> RLB
	if(getSwitchMotorsStat()==MOTORS_RLB) setSwitchMotorsStat(MOTORS_BRL);
	else setSwitchMotorsStat(getSwitchMotorsStat()+1);
	MotorWheel* temp=_wheelBack;
	_wheelBack=_wheelRight;
	_wheelRight=_wheelLeft;
	_wheelLeft=temp;
	return getSwitchMotorsStat();
}
unsigned char Omni3WD::switchMotorsRight() { // <-- BRL <-- LBR <--RLB
	if(getSwitchMotorsStat()==MOTORS_BRL) setSwitchMotorsStat(MOTORS_RLB);
	else setSwitchMotorsStat(getSwitchMotorsStat()-1);
	MotorWheel* temp=_wheelBack;
	_wheelBack=_wheelLeft;
	_wheelLeft=_wheelRight;
	_wheelRight=temp;
	return getSwitchMotorsStat();
} 
unsigned char Omni3WD::switchMotorsReset() {
	while(getSwitchMotorsStat()!=MOTORS_BRL) {
		switchMotorsLeft();
	}
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::getCarStat() const {
	return _carStat;
}
unsigned char Omni3WD::setCarStat(unsigned char carStat) {
    if(STAT_UNKNOWN<=carStat && carStat<=STAT_ROTATERIGHT)
        return _carStat=carStat;
    else
        return STAT_UNKNOWN;
}

unsigned int Omni3WD::setMotorAll(unsigned int speedMMPS,bool dir) {
	wheelBackSetSpeedMMPS(speedMMPS,dir);
	wheelRightSetSpeedMMPS(speedMMPS,dir);
	wheelLeftSetSpeedMMPS(speedMMPS,dir);
	return wheelBackGetSpeedMMPS();
}
unsigned int Omni3WD::setMotorAllStop() {
	return setMotorAll(0,DIR_ADVANCE);
}
unsigned int Omni3WD::setMotorAllAdvance(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_ADVANCE);
}
unsigned int Omni3WD::setMotorAllBackoff(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_BACKOFF);
}
unsigned int Omni3WD::setCarStop() {
	setCarStat(STAT_STOP);
	return setMotorAll(0,DIR_ADVANCE);
}
unsigned int Omni3WD::setCarAdvance(unsigned int speedMMPS) {
	setCarStat(STAT_ADVANCE);
	wheelBackSetSpeedMMPS(0,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	wheelLeftSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	return wheelRightGetSpeedMMPS(); 
}
unsigned int Omni3WD::setCarBackoff(unsigned int speedMMPS) {
	setCarStat(STAT_BACKOFF);
	wheelBackSetSpeedMMPS(0,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	wheelLeftSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	return wheelRightGetSpeedMMPS();
}
/*
unsigned int Omni3WD::setCarLeft(unsigned int speedMMPS) {
	setCarStat(STAT_LEFT);
	//wheelBackSetSpeedMMPS(speedMMPS+(speedMMPS>>1),DIR_ADVANCE);
	wheelBackSetSpeedMMPS(speedMMPS<<1,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	wheelLeftSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	return wheelRightGetSpeedMMPS();
}
unsigned int Omni3WD::setCarRight(unsigned int speedMMPS) {
	setCarStat(STAT_RIGHT);
	//wheelBackSetSpeedMMPS(speedMMPS+(speedMMPS>>1),DIR_BACKOFF);
	wheelBackSetSpeedMMPS(speedMMPS<<1,DIR_BACKOFF);
	wheelRightSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	wheelLeftSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	return wheelRightGetSpeedMMPS();
}
 */
unsigned int Omni3WD::setCarLeft(unsigned int speedMMPS) {
	setCarStat(STAT_LEFT);
	wheelBackSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS>>1,DIR_BACKOFF);
	wheelLeftSetSpeedMMPS(speedMMPS>>1,DIR_BACKOFF);
	return wheelBackGetSpeedMMPS();
}
unsigned int Omni3WD::setCarRight(unsigned int speedMMPS) {
	setCarStat(STAT_RIGHT);
	wheelBackSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	wheelRightSetSpeedMMPS(speedMMPS>>1,DIR_ADVANCE);
	wheelLeftSetSpeedMMPS(speedMMPS>>1,DIR_ADVANCE);
	return wheelBackGetSpeedMMPS();
}
unsigned int Omni3WD::setCarRotateLeft(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATELEFT);
	return setMotorAllBackoff(speedMMPS);
}
unsigned int Omni3WD::setCarRotateRight(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATERIGHT);
	return setMotorAllAdvance(speedMMPS);
}

unsigned int Omni3WD::getCarSpeedMMPS() const {
	unsigned int speedMMPS=wheelBackGetSpeedMMPS();
	if(wheelRightGetSpeedMMPS()>speedMMPS) 
	{
		speedMMPS=wheelRightGetSpeedMMPS();
	}
	if(wheelLeftGetSpeedMMPS()>speedMMPS) 
	{
		speedMMPS=wheelLeftGetSpeedMMPS();
	}
	return speedMMPS;
}
unsigned int Omni3WD::setCarSpeedMMPS(unsigned int speedMMPS,unsigned int ms) {
	unsigned int carStat=getCarStat();
	int currSpeed= getCarSpeedMMPS();  //
	int speedTemp = speedMMPS;
	unsigned int (Omni3WD::*carAction)(unsigned int speedMMPS);
	switch(carStat) {
		case STAT_UNKNOWN:	// no break here
		case STAT_STOP:
			return currSpeed;
		case STAT_ADVANCE:
			carAction=&Omni3WD::setCarAdvance; break;
		case STAT_BACKOFF:
			carAction=&Omni3WD::setCarBackoff; break;
		case STAT_LEFT:
			carAction=&Omni3WD::setCarLeft; break;
		case STAT_RIGHT:
			carAction=&Omni3WD::setCarRight; break;
		case STAT_ROTATELEFT:
			carAction=&Omni3WD::setCarRotateLeft; break;
		case STAT_ROTATERIGHT:
			carAction=&Omni3WD::setCarRotateRight; break;
	}

	if(ms<100 || abs(speedTemp-currSpeed)<10) {
		(this->*carAction)(speedMMPS);
		return getCarSpeedMMPS();
	}

	for(int time=0,speed=currSpeed;time<=ms;time+=50) {
		speed=abs(map(time,0,ms,currSpeed,speedTemp));
		(this->*carAction)(speed);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}
unsigned int Omni3WD::setCarSlow2Stop(unsigned int ms) {
	return setCarSpeedMMPS(0,ms);
}

unsigned int Omni3WD::wheelBackSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelBack->setSpeedMMPS(speedMMPS,dir);
}
/*unsigned int Omni3WD::wheelBackGetSpeedMMPS() const {
	return _wheelBack->getSpeedMMPS();
}*/
int Omni3WD::wheelBackGetSpeedMMPS() const {
	return _wheelBack->getSpeedMMPS();
}
unsigned int Omni3WD::wheelRightSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelRight->setSpeedMMPS(speedMMPS,dir);
}
/*unsigned int Omni3WD::wheelRightGetSpeedMMPS() const {
	return _wheelRight->getSpeedMMPS();
}*/
int Omni3WD::wheelRightGetSpeedMMPS() const {
	return _wheelRight->getSpeedMMPS();
}
unsigned int Omni3WD::wheelLeftSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLeft->setSpeedMMPS(speedMMPS,dir);
}
/*unsigned int Omni3WD::wheelLeftGetSpeedMMPS() const {
	return _wheelLeft->getSpeedMMPS();
}*/
int Omni3WD::wheelLeftGetSpeedMMPS() const {
	return _wheelLeft->getSpeedMMPS();
}
bool Omni3WD::PIDEnable(float kc,float taui,float taud,unsigned int interval) {
	return _wheelBack->PIDEnable(kc,taui,taud,interval) &&
			_wheelRight->PIDEnable(kc,taui,taud,interval) &&
			_wheelLeft->PIDEnable(kc,taui,taud,interval);
}
bool Omni3WD::PIDRegulate() {
	return _wheelBack->PIDRegulate() && _wheelRight->PIDRegulate() && _wheelLeft->PIDRegulate();
}
/*
void Omni3WD::delayMS(unsigned int ms,bool debug) {
	for(int i=0;i<ms;i+=SAMPLETIME) {
		PIDRegulate();
		if(debug && (i%500==0)) debugger();
		delay(SAMPLETIME);
	}
}
 */
void Omni3WD::delayMS(unsigned int ms,bool debug) {
	for(unsigned long endTime=millis()+ms;millis()<endTime;) {
		PIDRegulate();
		if(debug && (millis()%500==0)) debugger();
		if(endTime-millis()>=SAMPLETIME) delay(SAMPLETIME);
		else delay(endTime-millis());
	}
}

	// new one
void Omni3WD::demoActions(unsigned int speedMMPS,unsigned int duration,unsigned int uptime,bool debug) {
	unsigned int (Omni3WD::*carAction[])(unsigned int speedMMPS)={
		&Omni3WD::setCarAdvance,
		&Omni3WD::setCarBackoff,
		&Omni3WD::setCarLeft,
		&Omni3WD::setCarRight,
		&Omni3WD::setCarRotateLeft,
		&Omni3WD::setCarRotateRight
	};
	for(int i=0;i<6;++i) {
		(this->*carAction[i])(speedMMPS);
		setCarSpeedMMPS(speedMMPS,uptime);
		delayMS(duration,debug);
		setCarSlow2Stop(uptime);
	}
    setCarStop();
    delayMS(duration,debug);
    //switchMotorsLeft();
}
	// original
void Omni3WD::demoActions_Orginal(unsigned int speedMMPS,unsigned int ms,bool debug) {
	setCarAdvance(speedMMPS);
    delayMS(ms,debug);
    setCarBackoff(speedMMPS);
    delayMS(ms,debug);
    setCarLeft(speedMMPS);
    delayMS(ms,debug);
    setCarRight(speedMMPS);
    delayMS(ms,debug);
    setCarRotateLeft(speedMMPS);
    delayMS(ms,debug);
    setCarRotateRight(speedMMPS);
    delayMS(ms,debug);
    setCarStop();
    delayMS(1000,debug);
    switchMotorsLeft();
}

void Omni3WD::debugger(bool wheelBackDebug,bool wheelRightDebug,bool wheelLeftDebug) const {
	if(wheelBackDebug) _wheelBack->debugger();
	if(wheelRightDebug) _wheelRight->debugger();
	if(wheelLeftDebug) _wheelLeft->debugger();
}






