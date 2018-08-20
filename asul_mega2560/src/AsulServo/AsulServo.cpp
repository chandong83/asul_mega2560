
#include <inttypes.h>
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "AsulServo.h"


AsulServo::AsulServo()
{

    servoPin[ID_SERVO_LEFT_SHOULDER]  = 22;
    servoPin[ID_SERVO_LEFT_ARM]       = 24;
    servoPin[ID_SERVO_LEFT_FINGER]    = 26;
    servoPin[ID_SERVO_RIGHT_SHOULDER] = 28;
    servoPin[ID_SERVO_RIGHT_ARM]      = 30;
    servoPin[ID_SERVO_RIGHT_FINGER]   = 32;
    servoPin[ID_SERVO_NECK_TILT]      = 34;
    servoPin[ID_SERVO_NECK_PAN]       = 36;

    servoPos[ID_SERVO_LEFT_SHOULDER]  = 90;
    servoPos[ID_SERVO_LEFT_ARM]       = 90;
    servoPos[ID_SERVO_LEFT_FINGER]    = 90;
    servoPos[ID_SERVO_RIGHT_SHOULDER] = 90;
    servoPos[ID_SERVO_RIGHT_ARM]      = 90;
    servoPos[ID_SERVO_RIGHT_FINGER]   = 90;
    servoPos[ID_SERVO_NECK_TILT]      = 30;
    servoPos[ID_SERVO_NECK_PAN]       = 95;

}

void AsulServo::begin()
{
    for(int i=0;i<MAX_SERVOS;i++) {
        Servos[i].attach(servoPin[i]);
        Servos[i].write(servoPos[i]);
    }
}

void AsulServo::setPos(uint8_t id, uint8_t pos, uint16_t tm)
{
    if(id >= MAX_SERVOS)
        return;
    if(pos >= 180)
        pos = 180;
    Servos[id].write(pos);
    servoMoveTime[id] = tm;
}
