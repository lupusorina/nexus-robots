#include <EEPROM.h>

//#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
/********************************************************************/
/*


				        Power Switch

				        Sonar0x11

				 -------------------------
				/                         \
			       /		           \
			      /			            \
			M3   /			             \ M2
		       INT0 /			              \INT1
			   /			               \
			  /			                \
			 /			                 \
			 \			                 /
			  \			                /
		  	   \			               /
			    \			              /
		  Sonar0x12  \		  	             / Sonar0x13
			      \		                    /
			       \	                   /
				--------------------------
					    M1

 */


#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <MotorWheel.h>
#include <Omni3WD.h>

#include <SONAR.h>

/******************************************/
// SONAR

SONAR sonar11(0x11),sonar12(0x12),sonar13(0x13);

unsigned short distBuf[3];
void sonarsUpdate() {
    static unsigned char sonarCurr=1;
    if(sonarCurr==3) sonarCurr=1;
    else ++sonarCurr;
    if(sonarCurr==1) {        
        distBuf[1]=sonar12.getDist();        
        sonar12.trigger(); 
        sonar12.showDat();       
    } else if(sonarCurr==2) {
        distBuf[2]=sonar13.getDist();
        sonar13.trigger();
        sonar13.showDat();
    } else {
        distBuf[0]=sonar11.getDist();
        sonar11.trigger();
        sonar11.showDat();
    }
}

/*********************************************/

/*******************************************/
// Motors

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,6,7,&irq1);        // Pin9:PWM, Pin8:DIR, Pin6:PhaseA, Pin7:PhaseB

irqISR(irq2,isr2);
MotorWheel wheel2(10,11,14,15,&irq2);    // Pin10:PWM, Pin11:DIR, Pin14:PhaseA, Pin15:PhaseB

irqISR(irq3,isr3);
MotorWheel wheel3(3,2,4,5,&irq3);        // Pin3:PWM, Pin2:DIR, Pin4:PhaseA, Pin5:PhaseB

Omni3WD Omni(&wheel1,&wheel2,&wheel3);
/******************************************/

/******************************************/
// demo
unsigned long currMillis=0;
void demoWithSensors(unsigned int speedMMPS,unsigned int distance) {   
    if(millis()-currMillis>SONAR::duration) {
        currMillis=millis();
        sonarsUpdate();
    }   
   
    if(distBuf[1]<distance) {
        if(Omni.getCarStat()!=Omni3WD::STAT_RIGHT) Omni.setCarSlow2Stop(500);
            Omni.setCarRight(speedMMPS);
    } else if(distBuf[2]<distance) {
        if(Omni.getCarStat()!=Omni3WD::STAT_LEFT) Omni.setCarSlow2Stop(500);
            Omni.setCarLeft(speedMMPS);
    } else if(distBuf[0]<distance) {
        if(Omni.getCarStat()!=Omni3WD::STAT_ROTATERIGHT) Omni.setCarSlow2Stop(500);
            Omni.setCarRotateRight(speedMMPS);
    } else {
        if(Omni.getCarStat()!=Omni3WD::STAT_ADVANCE) Omni.setCarSlow2Stop(500);
            Omni.setCarAdvance(speedMMPS);
    }
    
    Omni.PIDRegulate();
}

/*****************************************/
// setup()
void setup() {
    TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
    TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
    
    SONAR::init(13);
    
    Omni.PIDEnable(0.26,0.02,0,10);
    
}

/****************************************/
// loop()
void loop() {
    demoWithSensors(100,30);
    //delay(500);
   //Omni.demoActions(100);
    //Serial.println("working");
}

