
// Ultrasonic Sensor


#define urm_MIN 4    // minimum mensurable distance: 4cm
#define urm_TX HIGH
#define urm_RX LOW
#define urm_bufSize 8
#define urm_total 6

//#define urm_baud 115200
#define urm_baud 19200
#define urm_duration 30

#define keyS7 3
//unsigned char keyS7=3;

//unsigned int urm_delay=160;
//unsigned int urm_delay=1;
#define urm_delay 1

unsigned char urm_rcvbuf[urm_bufSize];

unsigned short urm_disbuf[urm_total]={0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char urm_current=1;

//unsigned char urm_control=13; //HIGH:SEND, LOW:RECV
#define urm_control 13            // Pin13

unsigned char urm11Addr[]={0x55,0xaa,0xab,0x01,0x55,0x11,0x11};
unsigned char urm11Act[]={0x55,0xaa,0x11,0x00,0x01,0x11};
unsigned char urm11Get[]={0x55,0xaa,0x11,0x00,0x02,0x12};

unsigned char urm12Addr[]={0x55,0xaa,0xab,0x01,0x55,0x12,0x12};
unsigned char urm12Act[]={0x55,0xaa,0x12,0x00,0x01,0x12};
unsigned char urm12Get[]={0x55,0xaa,0x12,0x00,0x02,0x13};

unsigned char urm13Addr[]={0x55,0xaa,0xab,0x01,0x55,0x13,0x13};
unsigned char urm13Act[]={0x55,0xaa,0x13,0x00,0x01,0x13};
unsigned char urm13Get[]={0x55,0xaa,0x13,0x00,0x02,0x14};

unsigned char urm14Addr[]={0x55,0xaa,0xab,0x01,0x55,0x14,0x14};
unsigned char urm14Act[]={0x55,0xaa,0x14,0x00,0x01,0x14};
unsigned char urm14Get[]={0x55,0xaa,0x14,0x00,0x02,0x15};

unsigned char urm15Addr[]={0x55,0xaa,0xab,0x01,0x55,0x15,0x15};
 unsigned char urm15Act[]={0x55,0xaa,0x15,0x00,0x01,0x15};
 unsigned char urm15Get[]={0x55,0xaa,0x15,0x00,0x02,0x16};
 
 unsigned char urm16Addr[]={0x55,0xaa,0xab,0x01,0x55,0x16,0x16};
 unsigned char urm16Act[]={0x55,0xaa,0x16,0x00,0x01,0x16};
 unsigned char urm16Get[]={0x55,0xaa,0x16,0x00,0x02,0x17};


int urm_setMode(int mode);
unsigned char urm_sendCmd(unsigned char urm[],unsigned char size);
unsigned char urm_recvDat(unsigned char size=sizeof(urm_rcvbuf));
int urm_checksum(unsigned char size=sizeof(urm_rcvbuf));
void urm_showDat(unsigned char size=sizeof(urm_rcvbuf));
void urm_initAddr();
void urm_init();
int urm_action(unsigned char* act0,unsigned char act0_size,unsigned char* act1,unsigned char act1_size);
unsigned char urm_update(unsigned char total);
unsigned char urm_trigger(unsigned char* cmd,unsigned char size);
unsigned short urm_getDis(unsigned char* cmd,unsigned char size);



int urm_setMode(int mode) {		// HIGH:urm_TX, LOW:urm_RX
    digitalWrite(urm_control,mode);
    return mode;
}

unsigned char urm_sendCmd(unsigned char urm[],unsigned char size) {
    //digitalWrite(urm_control,HIGH);
    urm_setMode(urm_TX);
    //delay(10);
    //Serial.write(urm,size);
    for(int i=0;i<size;++i) {
        Serial.print(urm[i]);
    }
    return size;
}
unsigned char urm_recvDat(unsigned char size) {
    for(int i=0;i<sizeof(urm_rcvbuf);++i) {
        urm_rcvbuf[i]=0;
    }
    urm_setMode(urm_RX);
    for(int i=0,j=0;i<size&&j<5000;++j) {
        unsigned char ibyte=Serial.read();
        if(0<=ibyte && ibyte<0xff) {
            urm_rcvbuf[i++]=ibyte;
        }
    }
}
int urm_checksum(unsigned char size) {
    unsigned char sum=0;
    if(urm_rcvbuf[0]==0) return -1;
    for(int i=0;i<size-1;++i) {
        sum+=urm_rcvbuf[i];
    }
    if(sum!=urm_rcvbuf[size-1]) return -1;
    else return 0;
}
void urm_showDat(unsigned char size) {
    urm_setMode(urm_TX);
    for(int i=0;i<size;++i) {
        Serial.print(urm_rcvbuf[i],HEX);
        Serial.print(" ");
    }
    Serial.println("");
}
void urm_initAddr() {
    
    unsigned int startTime=millis();
    unsigned int delta=0;
    boolean got_key=false;

    while(digitalRead(keyS7)==LOW) {
        got_key=true;
        delay(100);
        Serial.println(delta=millis()-startTime,DEC);
        if(delta>=7000) return;        // no address will be set
    }
    if(got_key==true) {
        urm_setMode(urm_TX);
        Serial.print("Got keyS7:(ms) ");
        Serial.println(delta=millis()-startTime,DEC);
    }

    if(delta>=7000);
    else if(delta>=6000) urm_sendCmd(urm16Addr,sizeof(urm16Addr));
    else if(delta>=5000) urm_sendCmd(urm15Addr,sizeof(urm15Addr));
    else if(delta>=4000) urm_sendCmd(urm14Addr,sizeof(urm14Addr));
    else if(delta>=3000) urm_sendCmd(urm13Addr,sizeof(urm13Addr));
    else if(delta>=2000) urm_sendCmd(urm12Addr,sizeof(urm12Addr));
    else if(delta>=1000)  urm_sendCmd(urm11Addr,sizeof(urm11Addr));
    else got_key=false;
    if(got_key) {
        delay(1);
        //delayMicroseconds(200);
        urm_recvDat(7);
        urm_showDat(7);  
    }
    
}




void urm_init() {
    Serial.begin(urm_baud);
    pinMode(urm_control,OUTPUT);
    urm_setMode(urm_TX);


    pinMode(keyS7,INPUT);    // addressing in setup()
    delay(1000);
    urm_initAddr();   

     
    delay(2000);
}

unsigned char urm_trigger(unsigned char* cmd,unsigned char size) {
    urm_sendCmd(cmd,size);
    return 0;
}
unsigned short urm_getDis(unsigned char* cmd,unsigned char size) {
    urm_sendCmd(cmd,size);

    //delayMicroseconds(urm_delay);    // 150 - 240 us
    delay(urm_delay);                // 1ms

    urm_recvDat(urm_bufSize);
    urm_showDat();

    if(urm_checksum(urm_bufSize)==0)
        return (urm_rcvbuf[5]<<8)+urm_rcvbuf[6];
    return 0xffff; // not available distance
    
    
}
void urm_update() {
    if(urm_current==3) urm_current=1;
    else ++urm_current;
    if(urm_current==1) {        
        urm_disbuf[1]=urm_getDis(urm12Get,sizeof(urm12Get));        
        urm_disbuf[4]=urm_getDis(urm15Get,sizeof(urm15Get));        
        urm_trigger(urm11Act,sizeof(urm11Act));
        urm_trigger(urm14Act,sizeof(urm14Act));   
    } else if(urm_current==2) {
        urm_disbuf[2]=urm_getDis(urm13Get,sizeof(urm13Get));
        urm_disbuf[5]=urm_getDis(urm16Get,sizeof(urm16Get));
        urm_trigger(urm12Act,sizeof(urm12Act));
        urm_trigger(urm15Act,sizeof(urm15Act));
    } else {
        urm_disbuf[0]=urm_getDis(urm11Get,sizeof(urm11Get));
        urm_disbuf[3]=urm_getDis(urm14Get,sizeof(urm14Get));
        urm_trigger(urm13Act,sizeof(urm13Act));
        urm_trigger(urm16Act,sizeof(urm16Act));
    }
    return;
}

/*********************************************/


/*******************************************/
// Global
unsigned long currMillis=0;

/******************************************/
// mode
void demoWithSensors(unsigned int speedMMPS,unsigned int ms) {   
    if(millis()-currMillis>urm_duration) {
        currMillis=millis();
        urm_update();
    }   
    checkIRs();
}

/*****************************************/
// setup()
void setup() {
    urm_init();
    
}

/****************************************/
// loop()
void loop() {
    
    demoWithSensors(60,300);
}

