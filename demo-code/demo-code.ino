#define _NAMIKI_MOTOR
#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MatrixMath.h>
#include <math.h>

#define MOTOR1_PWM 9
#define MOTOR1_DIR 8
#define MOTOR1_ENCODER_A 6
#define MOTOR1_ENCODER_B 7

#define MOTOR2_PWM 10
#define MOTOR2_DIR 11
#define MOTOR2_ENCODER_A 14
#define MOTOR2_ENCODER_B 15

#define MOTOR3_PWM 3
#define MOTOR3_DIR 2
#define MOTOR3_ENCODER_A 4
#define MOTOR3_ENCODER_B 5

#define NR_OMNIWHEELS 3

float speed_kp = 0.7;
float speed_kd = 0;
float speed_ki = 0.008;
int sampling_time = SAMPLETIME; //ms

#define BODY_RADIUS  110 //mm
#define REDUCTION_RATIO_NAMIKI_MOTOR 80
#define WHEEL_RADIUS  24
#define WHEEL_CIRC  (WHEEL_RADIUS * 2 * M_PI)

int encoder_speed_wheel_back;
int encoder_speed_wheel_right;
int encoder_speed_wheel_left;

float matrix_speed_wheels_encoder[NR_OMNIWHEELS] = {0};
int i,j;

float back_wheel_speed, right_wheel_speed, left_wheel_speed;
float matrix_w_b[3][3];
float matrix_b_w[3][3];

unsigned long previous_time_ms;

// BRL
irqISR(irq1, isr1);
MotorWheel wheel_back(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, &irq1, REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

irqISR(irq2, isr2);
MotorWheel wheel_right(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, &irq2, REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

irqISR(irq3, isr3);
MotorWheel wheel_left(MOTOR3_PWM, MOTOR3_DIR, MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, &irq3, REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

Omni3WD Omni(&wheel_back, &wheel_right, &wheel_left);

void generate_matrix_wheel_body(float matrix_w_b[][3], float matrix_b_w[][3]){
    float angle_between_wheels[3];
    angle_between_wheels[0] = 180 * M_PI / 180; //rad
    angle_between_wheels[1] = (-60) * M_PI / 180; //rad
    angle_between_wheels[2] = 60 * M_PI / 180; //rad
    // see reference: https://github.com/cvra/CVRA-doc/blob/master/holonomic.pdf
    for (i = 0; i < 3; i++)
    {
        matrix_b_w[i][0] = -BODY_RADIUS;
        matrix_b_w[i][1] = sin(angle_between_wheels[i]);
        matrix_b_w[i][2] = -cos(angle_between_wheels[i]);
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            matrix_w_b[i][j] = matrix_b_w[i][j]; //create a copy
    Matrix.Invert((float*)matrix_w_b, NR_OMNIWHEELS); // this thing stores the inverted matrix in itself

}

bool setDirection(float value){
    return value > 0;
}

void set_speedMMPS(float desired_speed_body_frame[]){
    back_wheel_speed = right_wheel_speed = left_wheel_speed = 0;
    int i;
    for (i = 0; i <= 2; i++)
    {
      back_wheel_speed = back_wheel_speed + matrix_b_w[0][i] * desired_speed_body_frame[i];
      right_wheel_speed = right_wheel_speed + matrix_b_w[1][i] * desired_speed_body_frame[i];
      left_wheel_speed = left_wheel_speed + matrix_b_w[2][i] * desired_speed_body_frame[i];
    }
    wheel_back.setSpeedMMPS(abs(back_wheel_speed), setDirection(back_wheel_speed));
    wheel_right.setSpeedMMPS(abs(right_wheel_speed), setDirection(right_wheel_speed));
    wheel_left.setSpeedMMPS(abs(left_wheel_speed), setDirection(left_wheel_speed));
}

void generate_rot_matrix_body_to_inertial(float rotation_z_axis[][2], float theta){
    rotation_z_axis[0][0] = cos(theta);
    rotation_z_axis[0][1] = -sin(theta);
    rotation_z_axis[1][0] = sin(theta);
    rotation_z_axis[1][1] = cos(theta);
}

void get_speed_body_frame_from_encoderMMPS(float robot_speed_body_frame[], float *omega){
    matrix_speed_wheels_encoder[0] = Omni.wheelBackGetSpeedMMPS();
    matrix_speed_wheels_encoder[1] = Omni.wheelRightGetSpeedMMPS();
    matrix_speed_wheels_encoder[2] = Omni.wheelLeftGetSpeedMMPS();

    robot_speed_body_frame[0] = robot_speed_body_frame[1] = *omega = 0;

    for (i = 0; i <= 2; i++)
    {
      *omega = *omega + matrix_w_b[0][i] * matrix_speed_wheels_encoder[i];
      robot_speed_body_frame[0] = robot_speed_body_frame[0] + matrix_w_b[1][i] * matrix_speed_wheels_encoder[i];
      robot_speed_body_frame[1] = robot_speed_body_frame[1] + matrix_w_b[2][i] * matrix_speed_wheels_encoder[i];
    }
    Serial.println(robot_speed_body_frame[0], robot_speed_body_frame[1]);

}


void setup() {
    Serial.begin(115200);
    TCCR1B=TCCR1B&0xf8|0x01; // set the timers for PWM
    TCCR2B=TCCR2B&0xf8|0x01;
    Omni.PIDEnable(speed_kp, speed_ki, speed_kd, sampling_time);
    generate_matrix_wheel_body(matrix_w_b, matrix_b_w);
    previous_time_ms = millis();
 }


void robot_transform_body_to_inertial(float heading_angle, float vector[], float transformed_vector[]){
    float rotation_z_axis[2][2] = {0};
    generate_rot_matrix_body_to_inertial(rotation_z_axis, heading_angle);
    transformed_vector[0] = rotation_z_axis[0][0] * vector[0] + rotation_z_axis[0][1] * vector[1]; // x
    transformed_vector[1] = rotation_z_axis[1][0] * vector[0] + rotation_z_axis[1][1] * vector[1]; // y
}

void robot_integrate_speed(float position[], float *heading, float velocity[], float omega, float delta_t){
    *heading = *heading + omega*delta_t;
    position[0] = position[0] + velocity[0]*delta_t;
    position[1] = position[1] + velocity[1]*delta_t;
}

void drive_line_body_frame(float velocity_x, float velocity_y, float duration){
    float desired_speed_body_frame[3] = {0};
    desired_speed_body_frame[0] = 0; // rotations rad/sec
    desired_speed_body_frame[1] = velocity_x; // x directions, mm/sec
    desired_speed_body_frame[2] = velocity_y; // y direction, mm/sec
    set_speedMMPS(desired_speed_body_frame);

    Omni.delayMS(duration*1000, false);

}


float robot_position_inertial[2] = {0, 0};
float robot_heading_inertial = 0;

void loop() {
    unsigned long current_time_ms = millis();
    float robot_speed_inertial_frame[2] = {0};
    float robot_speed_body_frame[2] = {0};
    float omega;

    get_speed_body_frame_from_encoderMMPS(robot_speed_body_frame, &omega);

    unsigned long delta_t_ms = current_time_ms - previous_time_ms;
    float delta_t = (float)delta_t_ms / 1000;

    robot_transform_body_to_inertial(robot_heading_inertial, robot_speed_body_frame, robot_speed_inertial_frame);
    robot_integrate_speed(robot_position_inertial, &robot_heading_inertial, robot_speed_inertial_frame, omega, delta_t);
    previous_time_ms = current_time_ms;

    drive_line_body_frame(0, 50, 3);
    drive_line_body_frame(50, 0, 3);
    drive_line_body_frame(0, -50, 3);
    drive_line_body_frame(-50, 0, 3);




}
