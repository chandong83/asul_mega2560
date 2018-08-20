#ifndef _MOTOR_H_
#define _MOTOR_H_



#define L_MOTOR    0
#define R_MOTOR    1

#define TIRE_DIAMETER 43 //mm
#define TIRE_WITH     19 //mm
#define DISTANCE_BETWEEN_WHEELS 150 //mm

//uint8_t getTCount();
void motorBegin(void);
double motorGetDistance(uint8_t ch);
int32_t motorGetSpeed(uint8_t ch);
int32_t motorGetCounter(uint8_t ch);
int32_t motorGetGoalSpeed(uint8_t ch);
int32_t motorGetStartCounter(uint8_t ch);
int32_t motorGetPWM(uint8_t ch);
int32_t calculateEncCounter2RPM(int32_t count, int32_t checkTimeMs);
double calculateEncCounter2Distance(int32_t count);
double motorGetDistanceMiddlePoint();
double motorGetRotationAngle();
void motorClearDistanceInfo();
void motorSetSpeed(uint8_t ch, int16_t speed);
void motorSetPwm(uint8_t ch, int16_t pwm_data);
void motorMoveSpeed(int16_t left_speed, int16_t right_speed);
void moveMotor(int xAxis, int yAxis, uint8_t speed);
int32_t motorGetRPM(uint8_t ch);
#endif
