#ifndef Omni4WDAction_h
#define Omni4WDAction_h

#include <MotorWheel.h>
#include <Omni4WD.h>

/*
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
	};

	int (Omni4WD::*carAction[])(int speedMMPS)={
		&Omni4WD::setCarStop,
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

*/

#ifndef MAXID
#define MAXID 255
#endif

#ifndef MAXPRIO
#define MAXPRIO MAXID
#endif

#ifndef MAXSPEED
#define MAXSPEED 500
#endif


class Omni4WDAction {
public:
	bool isLastAct;
	//Omni4WDAction(Omni4WD* Omni);
	Omni4WDAction(Omni4WD* Omni,unsigned char carStat=Omni4WD::STAT_STOP,int speed=0,unsigned int duration=0,unsigned int uptime=0);
	Omni4WDAction(unsigned char carStat,int speed=0,unsigned int duration=0,unsigned int uptime=0);
	~Omni4WDAction();
	static Omni4WDAction* getFirstAction();
	static Omni4WDAction* setFirstAction(Omni4WDAction* action);

	Omni4WDAction* getNextAction();
	Omni4WDAction* setNextAction(Omni4WDAction* action);

	static unsigned char getNextID();
	static unsigned char setNextID(unsigned char id);
	static unsigned char incNextID();
	static unsigned char decNextID();
	static unsigned char resetNextID();

	unsigned char getID() const;
	unsigned char setID(unsigned char id);

	unsigned char getCarStat() const;
	unsigned char setCarStat(unsigned char carStat);

	int getCarSpeedMMPS() const;
	int setCarSpeedMMPS(int speedMMPS);
	
	unsigned int getUptime() const;
	unsigned int setUptime(unsigned int uptime);

	unsigned int getDuration() const;
	unsigned int setDuration(unsigned int duration);

	unsigned char getPrio() const;
	unsigned char setPrio(unsigned char prio);
	unsigned char incPrio();
	unsigned char decPrio();
	unsigned char updatePrioAll();

	unsigned char getStat() const;
	unsigned char setStat(unsigned char stat);
	unsigned char markQueuing();
	unsigned char markActing();
	unsigned char markDone();
	unsigned char markDel();

	Omni4WDAction* add(Omni4WDAction* action);
	//Omni4WDAction* add(Omni4WDAction& action);
	Omni4WDAction* findActing();
	Omni4WDAction* find1stPrio();
	Omni4WDAction* findNExec();
	Omni4WDAction* findNReuse(unsigned char carStat,int speed=0,unsigned int duration=0,unsigned int uptime=0);
	unsigned char exec();
	unsigned char modify(unsigned char carStat,int speed=0,unsigned int duration=0,unsigned int uptime=0);
	unsigned char del();
	unsigned char delAll();
	unsigned char halt();			// halt an queuing or acting action
	unsigned char haltAll();
	unsigned char active();			// active an halted action
	unsigned char activeAll();

	unsigned char countActive() const;	// STAT_QUEUING, STAT_ACTING and STAT_HALTED
	unsigned char countAll() const;		// All Omni4WDAction objects
	
	unsigned char Kill(bool b_SlowStop);
	unsigned char Start();

	enum {
		STAT_UNKNOWN,
		STAT_QUEUING,
		STAT_ACTING,
		STAT_DONE,
		STAT_FAILED,
		STAT_HALTED,
	};
	void debugger() const;
	void delAction(bool isAll = false);
	bool isNull();
private:
	Omni4WDAction();	// disable default constructor
	unsigned char init();
	unsigned char reset();

	static Omni4WD* getOmni4WD();
	static Omni4WD* setOmni4WD(Omni4WD* Omni4WD);

	static Omni4WD* _Omni;	// Omni4WD Chassis
	static unsigned char _nextID;	// next ID available
	static Omni4WDAction* _firstAction;
	Omni4WDAction* _nextAction;

	unsigned char _id;
	unsigned char _carStat;
	int _speedMMPS;
	unsigned int _duration;
	unsigned int _uptime;
	unsigned char _prio;
	unsigned char _stat;
	
	unsigned long _startTime;
	unsigned long _endTime;
	
	//static int initCarActions();
	//static int (Omni4WD::*carActions[12])(int speedMMPS);	

};

#endif


