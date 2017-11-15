#include <Omni4WD.h>




Omni4WD::Omni4WD(MotorWheel* wheelUL,MotorWheel* wheelLL,
			MotorWheel* wheelLR,MotorWheel* wheelUR,unsigned int wheelspan):
			_wheelUL(wheelUL),_wheelLL(wheelLL),
			_wheelLR(wheelLR),_wheelUR(wheelUR),_wheelspan(wheelspan) {
	setSwitchMotorsStat(MOTORS_FB);
}
unsigned char Omni4WD::getSwitchMotorsStat() const {
	return _switchMotorsStat;
}
unsigned char Omni4WD::setSwitchMotorsStat(unsigned char switchMotorsStat) {
	if(MOTORS_FB<=switchMotorsStat && switchMotorsStat<=MOTORS_BF)
		_switchMotorsStat=switchMotorsStat;
	return getSwitchMotorsStat();
}
unsigned char Omni4WD::switchMotors() {
	if(getSwitchMotorsStat()==MOTORS_FB) {
		setSwitchMotorsStat(MOTORS_BF);
	} else {
		setSwitchMotorsStat(MOTORS_FB);
	}
	MotorWheel* temp=_wheelUL;
	_wheelUL=_wheelLR;
	_wheelLR=temp;
	temp=_wheelLL;
	_wheelLL=_wheelUR;
	_wheelUR=temp;

	return getSwitchMotorsStat();
}
unsigned char Omni4WD::switchMotorsReset() {
	if(getSwitchMotorsStat()==MOTORS_BF) switchMotors();
	return getSwitchMotorsStat();
}

unsigned int Omni4WD::getWheelspan() const {
	return _wheelspan;
}

unsigned char Omni4WD::getCarStat() const {
    return _carStat;
}
unsigned char Omni4WD::setCarStat(unsigned char carStat) {
    if(STAT_UNKNOWN<=carStat && carStat<=STAT_UPPERRIGHT)
        return _carStat=carStat;
    else
        return STAT_UNKNOWN;
}

unsigned int Omni4WD::setMotorAll(unsigned int speedMMPS,bool dir) {
	wheelULSetSpeedMMPS(speedMMPS,dir);
	wheelLLSetSpeedMMPS(speedMMPS,dir);
	wheelLRSetSpeedMMPS(speedMMPS,dir);
	wheelURSetSpeedMMPS(speedMMPS,dir);
	return wheelULGetSpeedMMPS();
}
unsigned int Omni4WD::setMotorAllStop() {
	return setMotorAll(0,DIR_ADVANCE);
}
unsigned int Omni4WD::setMotorAllAdvance(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_ADVANCE);
}
unsigned int Omni4WD::setMotorAllBackoff(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_BACKOFF);
}
unsigned int Omni4WD::setCarStop(unsigned int mm) {
	setCarStat(STAT_STOP);
	return setMotorAllStop();
}

// 201208
// Mecanum Wheel Control Summary.pdf
// V1=Vty+Vtx-w(a+b)
// V2=Vty-Vtx-w(a+b)
// V3=Vty+Vtx+w(a+b)
// V4=Vty-Vtx+w(a+b)
// To implememt Omni4WD::setCarMove(), MotorWheel::setSpeedMMPS() was re-written as plus-minus/direction sensitive.
int Omni4WD::setCarMove(int speedMMPS,float rad,float omega) {
	//wheelULSetSpeedMMPS(speedMMPS*sin(rad)+speedMMPS*cos(rad)-omega*WHEELSPAN);
	//wheelLLSetSpeedMMPS(speedMMPS*sin(rad)-speedMMPS*cos(rad)-omega*WHEELSPAN);
	//wheelLRSetSpeedMMPS(speedMMPS*sin(rad)+speedMMPS*cos(rad)+omega*WHEELSPAN);
	//wheelURSetSpeedMMPS(speedMMPS*sin(rad)-speedMMPS*cos(rad)+omega*WHEELSPAN);

	wheelULSetSpeedMMPS(speedMMPS*sin(rad)+speedMMPS*cos(rad)-omega*WHEELSPAN);
	wheelLLSetSpeedMMPS(speedMMPS*sin(rad)-speedMMPS*cos(rad)-omega*WHEELSPAN);
	wheelLRSetSpeedMMPS(-(speedMMPS*sin(rad)+speedMMPS*cos(rad)+omega*WHEELSPAN));
	wheelURSetSpeedMMPS(-(speedMMPS*sin(rad)-speedMMPS*cos(rad)+omega*WHEELSPAN));

	return getCarSpeedMMPS();
}

int Omni4WD::setCarAdvance(int speedMMPS) {
	setCarStat(STAT_ADVANCE);
	//wheelULSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	//wheelLLSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	//wheelLRSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	//wheelURSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	//return wheelULGetSpeedMMPS();
	return setCarMove(speedMMPS,PI/2,0);
}
int Omni4WD::setCarBackoff(int speedMMPS) {
	setCarStat(STAT_BACKOFF);
	return setCarMove(speedMMPS,PI*3/2,0);
}
int Omni4WD::setCarLeft(int speedMMPS) {
	setCarStat(STAT_LEFT);
	//speedMMPS*=2;
	//wheelULSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	//wheelLLSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	//wheelLRSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	//wheelURSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	//return wheelULGetSpeedMMPS();
	return setCarMove(speedMMPS,PI,0);
}
int Omni4WD::setCarRight(int speedMMPS) {
	setCarStat(STAT_RIGHT);
	return setCarMove(speedMMPS,0,0);
}
int Omni4WD::setCarUpperLeft(int speedMMPS) {
	setCarStat(STAT_UPPERLEFT);
	//speedMMPS*=2;
	//wheelULSetSpeedMMPS(0,DIR_ADVANCE);
	//wheelLLSetSpeedMMPS(speedMMPS,DIR_ADVANCE);
	//wheelLRSetSpeedMMPS(0,DIR_ADVANCE);
	//wheelURSetSpeedMMPS(speedMMPS,DIR_BACKOFF);
	//return wheelLLGetSpeedMMPS();
	return setCarMove(speedMMPS,PI*3/4,0);
}
int Omni4WD::setCarLowerLeft(int speedMMPS) {
	setCarStat(STAT_LOWERLEFT);
	return setCarMove(speedMMPS,PI*5/4,0);
}
int Omni4WD::setCarLowerRight(int speedMMPS) {
	setCarStat(STAT_LOWERRIGHT);
	return setCarMove(speedMMPS,PI*7/4,0);
}
int Omni4WD::setCarUpperRight(int speedMMPS) {
	setCarStat(STAT_UPPERRIGHT);
	return setCarMove(speedMMPS,PI/4,0);
}

float Omni4WD::setCarRotate(float omega) {
	return setCarMove(0,0,omega);
}
int Omni4WD::setCarRotateLeft(int speedMMPS) {
	setCarStat(STAT_ROTATELEFT);
	return setCarRotate(speedMMPS/(sqrt(pow(getWheelspan()/2,2)*2)));
}
int Omni4WD::setCarRotateRight(int speedMMPS) {
	setCarStat(STAT_ROTATERIGHT);
	return setCarRotate(-speedMMPS/(sqrt(pow(getWheelspan()/2,2)*2)));
}


/*
unsigned int Omni4WD::getCarSpeedMMPS() const {
	unsigned int speedMMPS=wheelULGetSpeedMMPS();
	if(wheelLLGetSpeedMMPS()>speedMMPS) speedMMPS=wheelLLGetSpeedMMPS();
	if(wheelLRGetSpeedMMPS()>speedMMPS) speedMMPS=wheelLRGetSpeedMMPS();
	if(wheelURGetSpeedMMPS()>speedMMPS) speedMMPS=wheelURGetSpeedMMPS();
	return speedMMPS;
}
 */
// 201208
float Omni4WD::getCarSpeedRad() const {	// Omega
	unsigned char carStat=getCarStat();
	switch(carStat) {
		case STAT_STOP:
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_LEFT:
		case STAT_RIGHT:
		case STAT_LOWERLEFT:
		case STAT_UPPERRIGHT:
		case STAT_UPPERLEFT:
		case STAT_LOWERRIGHT:
			return 0;break;
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return wheelULGetSpeedMMPS()/sqrt(pow(getWheelspan()/2,2)*2*2); break;
			//return abs(wheelULGetSpeedMMPS()/getWheelspan()); break;
		case STAT_UNKNOWN:	// Not implemented yet
			break;
	}
	return 0;
}
int Omni4WD::getCarSpeedMMPS() const {
	unsigned char carStat=getCarStat();
	switch(carStat) {
		case STAT_STOP:
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_LEFT:
		case STAT_RIGHT:
			return abs(wheelULGetSpeedMMPS()); break;
		case STAT_LOWERLEFT:
		case STAT_UPPERRIGHT:
			return abs(wheelULGetSpeedMMPS()/sqrt(2)); break;
		case STAT_UPPERLEFT:
		case STAT_LOWERRIGHT:
			return abs(wheelLLGetSpeedMMPS()/sqrt(2)); break;
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return abs(getCarSpeedRad()*sqrt(pow(getWheelspan()/2,2)*2)); break;
		case STAT_UNKNOWN:	// Not implemented yet
			break;
	}
	return 0;
}


int Omni4WD::setCarSpeedMMPS(int speedMMPS,unsigned int ms) {
	unsigned int carStat=getCarStat();
	int currSpeed=getCarSpeedMMPS();

	int (Omni4WD::*carAction)(int speedMMPS);
	switch(carStat) {
		case STAT_UNKNOWN:	// no break here
		case STAT_STOP:
			return currSpeed;
		case STAT_ADVANCE:
			carAction=&Omni4WD::setCarAdvance; break;
		case STAT_BACKOFF:
			carAction=&Omni4WD::setCarBackoff; break;
		case STAT_LEFT:
			carAction=&Omni4WD::setCarLeft; break;
		case STAT_RIGHT:
			carAction=&Omni4WD::setCarRight; break;
		case STAT_ROTATELEFT:
			carAction=&Omni4WD::setCarRotateLeft; break;
		case STAT_ROTATERIGHT:
			carAction=&Omni4WD::setCarRotateRight; break;
		case STAT_UPPERLEFT:
			carAction=&Omni4WD::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&Omni4WD::setCarLowerLeft; break;
		case STAT_LOWERRIGHT:
			carAction=&Omni4WD::setCarLowerRight; break;
		case STAT_UPPERRIGHT:
			carAction=&Omni4WD::setCarUpperRight; break;
	}

	if(ms<100 || abs(speedMMPS-currSpeed)<10) {
		(this->*carAction)(speedMMPS);
		return getCarSpeedMMPS();
	}

	for(int time=20,speed=currSpeed;time<=ms;time+=20) {
		speed=map(time,0,ms,currSpeed,speedMMPS);
		(this->*carAction)(speed);
		delayMS(20);
	}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}
int Omni4WD::setCarSlow2Stop(unsigned int ms) {
	return setCarSpeedMMPS(0,ms);
}

unsigned int Omni4WD::wheelULSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelUL->setSpeedMMPS(speedMMPS,dir);
}
int Omni4WD::wheelULSetSpeedMMPS(int speedMMPS) { // direction sensitive, 201208
	return _wheelUL->setSpeedMMPS(speedMMPS);
}
int Omni4WD::wheelULGetSpeedMMPS() const {
	return _wheelUL->getSpeedMMPS();
}
unsigned int Omni4WD::wheelLLSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLL->setSpeedMMPS(speedMMPS,dir);
}
int Omni4WD::wheelLLSetSpeedMMPS(int speedMMPS) { // direction sensitive, 201208
	return _wheelLL->setSpeedMMPS(speedMMPS);
}
int Omni4WD::wheelLLGetSpeedMMPS() const {
	return _wheelLL->getSpeedMMPS();
}
unsigned int Omni4WD::wheelLRSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLR->setSpeedMMPS(speedMMPS,dir);
}
int Omni4WD::wheelLRSetSpeedMMPS(int speedMMPS) { // direction sensitive, 201208
	return _wheelLR->setSpeedMMPS(speedMMPS);
}
int Omni4WD::wheelLRGetSpeedMMPS() const {
	return _wheelLR->getSpeedMMPS();
}
unsigned int Omni4WD::wheelURSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelUR->setSpeedMMPS(speedMMPS,dir);
}
int Omni4WD::wheelURSetSpeedMMPS(int speedMMPS) { // direction sensitive, 201208
	return _wheelUR->setSpeedMMPS(speedMMPS);
}
int Omni4WD::wheelURGetSpeedMMPS() const {
	return _wheelUR->getSpeedMMPS();
}
bool Omni4WD::PIDEnable(float kc,float taui,float taud,unsigned int interval) {
	return _wheelUL->PIDEnable(kc,taui,taud,interval) &&
			_wheelLL->PIDEnable(kc,taui,taud,interval) &&
			_wheelLR->PIDEnable(kc,taui,taud,interval) &&
			_wheelUR->PIDEnable(kc,taui,taud,interval);
}
bool Omni4WD::PIDDisable() {
	setCarStat(STAT_UNKNOWN);
	_wheelUL->PIDDisable(); _wheelUL->runPWM(0,DIR_ADVANCE);
	_wheelLL->PIDDisable(); _wheelLL->runPWM(0,DIR_ADVANCE);
	_wheelLR->PIDDisable(); _wheelLR->runPWM(0,DIR_ADVANCE);
	_wheelUR->PIDDisable(); _wheelUR->runPWM(0,DIR_ADVANCE);
	return false;
}
bool Omni4WD::PIDGetStatus() {
	return _wheelUL->PIDGetStatus() && _wheelLL->PIDGetStatus() && _wheelLR->PIDGetStatus() && _wheelUR->PIDGetStatus();
}
float Omni4WD::PIDGetP_Param() {
	return _wheelUL->GetP_Param();
}
float Omni4WD::PIDGetI_Param() {
	return _wheelUL->GetI_Param();
}
float Omni4WD::PIDGetD_Param() {
	return _wheelUL->GetD_Param();
}
bool Omni4WD::PIDRegulate() {
	return _wheelUL->PIDRegulate() && _wheelLL->PIDRegulate() && _wheelLR->PIDRegulate() && _wheelUR->PIDRegulate();
}
/*
void Omni4WD::delayMS(unsigned int ms,unsigned int slot,bool debug) {
	for(int i=0;i<ms;i+=slot) {
		PIDRegulate();
		if(debug && (i%500==0)) debugger();
		delay(slot);
	}
}
 */
void Omni4WD::delayMS(unsigned int ms,bool debug,unsigned char* actBreak) {		// 201209
	for(unsigned long endTime=millis()+ms;millis()<endTime;) 
	{
		if(actBreak) return;
		PIDRegulate();
		if(debug && (millis()%500==0)) debugger();
		if(endTime-millis()>=SAMPLETIME) delay(SAMPLETIME);
		else delay(endTime-millis());
	}
}
		// new one
void Omni4WD::demoActions(unsigned int speedMMPS,unsigned int duration,
		unsigned int uptime,bool debug) {
		int (Omni4WD::*carAction[])(int speedMMPS)={
		&Omni4WD::setCarAdvance,
		&Omni4WD::setCarBackoff,
		&Omni4WD::setCarLeft,
		&Omni4WD::setCarRight,
		&Omni4WD::setCarUpperLeft,
		&Omni4WD::setCarLowerRight,
		&Omni4WD::setCarLowerLeft,
		&Omni4WD::setCarUpperRight,
		&Omni4WD::setCarRotateLeft,
		&Omni4WD::setCarRotateRight
	};

	for(int i=0;i<10;++i) {
		(this->*carAction[i])(0); // default parameters not available in function pointer
		setCarSpeedMMPS(speedMMPS,uptime);
		delayMS(duration,debug);
		setCarSlow2Stop(uptime);
	}
	setCarStop();
	delayMS(duration);
	//switchMotors();
}
/*		// original
void Omni4WD::demoActions(unsigned int speedMMPS,unsigned int uptime,
							unsigned int duration,bool debug) {
	setCarAdvance();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarBackoff();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarLeft();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarRight();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarUpperLeft();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarLowerRight();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarLowerLeft();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarUpperRight();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarRotateLeft();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarRotateRight();
	setCarSpeedMMPS(speedMMPS,uptime);
	delayMS(duration,debug);
	setCarSlow2Stop(uptime);
	setCarStop();
	delayMS(duration,debug);
	switchMotors();
}
 */
void Omni4WD::debugger(bool wheelULDebug,bool wheelLLDebug,bool wheelLRDebug,bool wheelURDebug) const {
	if(wheelULDebug) _wheelUL->debugger();
	if(wheelLLDebug) _wheelLL->debugger();
	if(wheelLRDebug) _wheelLR->debugger();
	if(wheelURDebug) _wheelUR->debugger();
}




