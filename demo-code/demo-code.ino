#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include "demo-code.h"

irqISR(irq1, isr1);
MotorWheel wheelBack(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, &irq1);

irqISR(irq2, isr2);
MotorWheel wheelRight(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, &irq2);

irqISR(irq3, isr3);
MotorWheel wheelLeft(MOTOR3_PWM, MOTOR3_DIR, MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, &irq3);

Omni3WD Omni(&wheelBack, &wheelRight, &wheelLeft);

float speed_kp = 0.26;
float speed_kd = 0.02;
float speed_ki = 0.10;
int sampling_time = 10; //ms
void setup() {
    Serial.begin(9600);
    Serial.println("motor stat");
    TCCR1B=TCCR1B&0xf8|0x01; 
    TCCR2B=TCCR2B&0xf8|0x01;
    Omni.PIDEnable(speed_kp, speed_kd, speed_ki, sampling_time);
}

int encoder_speed_wheel_back;
int encoder_speed_wheel_right;
int encoder_speed_wheel_left;
unsigned int speed_set = 20;

void loop() {
  
    wheelBack.setSpeedMMPS(speed_set, false);
    wheelRight.setSpeedMMPS(speed_set, false);
    wheelLeft.setSpeedMMPS(speed_set, false);

    Omni.delayMS(sampling_time, true);
    encoder_speed_wheel_back = Omni.wheelBackGetSpeedMMPS();
    encoder_speed_wheel_right = Omni.wheelRightGetSpeedMMPS();
    encoder_speed_wheel_left = Omni.wheelLeftGetSpeedMMPS();

    Serial.print(" Wheel BACK = ");
    Serial.print(encoder_speed_wheel_back);
    Serial.print(" Wheel RIGHT = ");
    Serial.print(encoder_speed_wheel_right);
    Serial.print(" Wheel LEFT = ");
    Serial.print(encoder_speed_wheel_left);
    Serial.println();

}
