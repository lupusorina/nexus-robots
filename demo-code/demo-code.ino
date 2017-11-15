#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include "demo-code.h"
#include <MatrixMath.h>
#include <math.h>

#define MOTOR1_PWM 9
#define MOTOR1_DIR 8
#define MOTOR1_ENCODER_A 7
#define MOTOR1_ENCODER_B 6

#define MOTOR2_PWM 10
#define MOTOR2_DIR 11
#define MOTOR2_ENCODER_A 15
#define MOTOR2_ENCODER_B 14

#define MOTOR3_PWM 3
#define MOTOR3_DIR 2
#define MOTOR3_ENCODER_A 5
#define MOTOR3_ENCODER_B 4

#define NR_OMNIWHEELS 3

float speed_kp = 4;
float speed_kd = 0;
float speed_ki = 0.008;
int sampling_time = SAMPLETIME; //ms

float wheel_radius = 24; //mm
float body_radius = 90; //mm

float beta[3];


// BRL
irqISR(irq1, isr1);
MotorWheel wheelBack(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, &irq1);

irqISR(irq2, isr2);
MotorWheel wheelRight(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, &irq2);

irqISR(irq3, isr3);
MotorWheel wheelLeft(MOTOR3_PWM, MOTOR3_DIR, MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, &irq3);

Omni3WD Omni(&wheelBack, &wheelRight, &wheelLeft);

void generate_matrix_wheel_body(float matrix_w_b[][3], float matrix_b_w[][3]){
    float beta[3];
    beta[0] = 180 * M_PI / 180; //rad
    beta[1] = (-60) * M_PI / 180; //rad
    beta[2] = 60 * M_PI / 180; //rad

    int i, j;
    for (i = 0; i < 3; i++){
        matrix_b_w[i][0] = -body_radius;
        matrix_b_w[i][1] = sin(beta[i]);
        matrix_b_w[i][2] = -cos(beta[i]);
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            matrix_w_b[i][j] = matrix_w_b[i][j];
    Matrix.Invert((float*)matrix_w_b, NR_OMNIWHEELS); // this thing stores the inverted matrix in itself 
    Matrix.Print((float*)matrix_b_w, 3, 3, "M B_W");

}

bool setDirection(float value){
    return value > 0;
}

float back_wheel_speed, right_wheel_speed, left_wheel_speed;
float matrix_w_b[3][3]={0};
float matrix_b_w[3][3]={0};

void setSpeedMMPS(float desired_speed_body_frame[]){
    back_wheel_speed = right_wheel_speed = left_wheel_speed = 0;
    int i;
    for (i = 0; i <= 2; i++)
    {
      back_wheel_speed = back_wheel_speed + matrix_b_w[0][i] * desired_speed_body_frame[i];
      right_wheel_speed = right_wheel_speed + matrix_b_w[1][i] * desired_speed_body_frame[i];
      left_wheel_speed = left_wheel_speed + matrix_b_w[2][i] * desired_speed_body_frame[i];
    }
    wheelBack.setSpeedMMPS(abs(back_wheel_speed), setDirection(back_wheel_speed));
    wheelRight.setSpeedMMPS(abs(right_wheel_speed), setDirection(right_wheel_speed));
    wheelLeft.setSpeedMMPS(abs(left_wheel_speed), setDirection(left_wheel_speed));

    Serial.print("Desired wheel speeds ");
    Serial.print(back_wheel_speed);
    Serial.print(" ");
    Serial.print(left_wheel_speed);
    Serial.print(" ");
    Serial.print(right_wheel_speed);
    Serial.println();
}



void setup() {
    Serial.begin(9600);
    Serial.println("motor stat");
    TCCR1B=TCCR1B&0xf8|0x01;
    TCCR2B=TCCR2B&0xf8|0x01;
    Omni.PIDEnable(speed_kp, speed_ki, speed_kd, sampling_time);
    generate_matrix_wheel_body(matrix_w_b, matrix_b_w);
 }

int encoder_speed_wheel_back;
int encoder_speed_wheel_right;
int encoder_speed_wheel_left;
unsigned int speed_set = 20;
float matrix_speed_wheels_encoder[NR_OMNIWHEELS] = {0};
int i,j;
float omega_body_frame, vx_body_frame, vy_body_frame;
float desired_speed_body_frame[NR_OMNIWHEELS] = {0};
void loop() {

    omega_body_frame = 0;
    vx_body_frame = 0;
    vy_body_frame = 0;
    for (i = 0; i <= 2; i++)
    {
      omega_body_frame = omega_body_frame + matrix_w_b[0][i] * matrix_speed_wheels_encoder[i];
      vx_body_frame = vx_body_frame + matrix_w_b[1][i] * matrix_speed_wheels_encoder[i];
      vy_body_frame = vy_body_frame + matrix_w_b[2][i] * matrix_speed_wheels_encoder[i];
    }

    //setpoint
    desired_speed_body_frame[0] = 0; // rotations rad/sec
    desired_speed_body_frame[1] = 0; // x directions, mm/sec
    desired_speed_body_frame[2] = 60; // y direction, mm/sec

    setSpeedMMPS(desired_speed_body_frame);

    Omni.delayMS(sampling_time, true);

    matrix_speed_wheels_encoder[0] = Omni.wheelBackGetSpeedMMPS();
    matrix_speed_wheels_encoder[1] = Omni.wheelRightGetSpeedMMPS();
    matrix_speed_wheels_encoder[2] = Omni.wheelLeftGetSpeedMMPS();


}
