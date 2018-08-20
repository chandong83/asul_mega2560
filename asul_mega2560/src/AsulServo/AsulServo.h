#include <Servo.h>
#ifndef __ASUL_SERVO_H__
#define __ASUL_SERVO_H__

enum {
    ID_SERVO_LEFT_SHOULDER = 0,
    ID_SERVO_LEFT_ARM,
    ID_SERVO_LEFT_FINGER,
    ID_SERVO_RIGHT_SHOULDER,
    ID_SERVO_RIGHT_ARM,
    ID_SERVO_RIGHT_FINGER,
    ID_SERVO_NECK_TILT,
    ID_SERVO_NECK_PAN,
};
#define MAX_SERVOS 8

class AsulServo
{
public:
   AsulServo();
   void begin();
   void setPos(uint8_t id, uint8_t pos, uint16_t tm);
private:


    Servo Servos[MAX_SERVOS];

    int servoPos[MAX_SERVOS];
    int servoPin[MAX_SERVOS];
    int servoPosMin[MAX_SERVOS];
    int servoPosMax[MAX_SERVOS];
    int servoMoveTime[MAX_SERVOS];

};

#endif
