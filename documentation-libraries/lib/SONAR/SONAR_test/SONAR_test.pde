#include <SONAR.h>


SONAR s11=SONAR(0x11);
//SONAR s12(0x12);

void setup() {
    SONAR::init();
    delay(100);
    //s11.setAddr(0x11); // Warning!! It will set hardware address of all connected SONAR sensors to the given value
}

void loop() {
    s11.trigger();
    //s12.trigger();
    delay(SONAR::duration);
    Serial.println(s11.getDist(),DEC);
    s11.showDat();
    //Serial.println(s12.getDist(),DEC);
    Serial.println(s11.getTemp(),DEC);
    //Serial.println(s12.getTemp(),DEC);
    delay(500);
}


