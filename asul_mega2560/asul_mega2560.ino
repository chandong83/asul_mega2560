#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "motor.h"
#include "battery.h"
#include "src/AsulServo/AsulServo.h"
#include "src/buzzer/OttoSounds.h"
#include "src/Bluetooth/Bluetooth.h"

#define PIN_Buzzer  45


battery myBattery = battery(0, 3.3, 1.2);
OttoSounds mySounds = OttoSounds();
Bluetooth myBT = Bluetooth();
AsulServo myServo = AsulServo();

#define ctlSerial Serial2
void setup() {
  motorBegin();
  motorMoveSpeed(0, 0);
  myServo.begin();
  mySounds.begin();  
  mySounds.sing(S_superHappy);
  delay(300);
  //myServo.setPos(ID_SERVO_NECK_TILT, 30, 0);
  
  Serial.begin(9600);
  myBT.begin(9600);

  pinMode(13,OUTPUT);

  motorClearDistanceInfo();
}

uint8_t dirFg=0;
uint32_t prevPrintTm=0;
uint32_t prevMotorTm=0;
uint8_t motorMode=0;
void loop() {
    uint32_t newTm;
    uint8_t cmd=0;
    uint8_t sub=0;
    int jX, jY;
    if(myBT.update())
    {
        myBT.getData(&cmd, &sub);
        if(cmd == 3)
        {
            if(sub == 1)
            {
                  motorMoveSpeed(50, 50);
            }
            else if(sub == 2)
            {
                  motorMoveSpeed(-50, -50);
            }
            else if(sub == 3)
            {
                  motorMoveSpeed(50, -50);
            }
            else if(sub == 4)
            {
                  motorMoveSpeed(-50, 50);
            }
            else
            {
                  motorMoveSpeed(0, 0);
            }
        }
        else if(cmd == 4)
        {
            if(sub == 1)
                mySounds.sing(0);
            else if(sub == 2)
                mySounds.sing(1);
        }
        else if(cmd == 5)
        {
            myBT.getJoyData(&jX, &jY);
            moveMotor(jX, jY, 255);
        }
    }
    
    if(myBattery.updateBattery() == 1){
        Serial.print("Battery Power : ");
        Serial.print(myBattery.readBattery());
        Serial.println("V");
    }

    newTm = millis();
    //5sec
    if((newTm - prevMotorTm) >= 5000)
    {
     // motorSetPwm(L_MOTOR, 255);
      prevMotorTm = newTm;
      switch(motorMode)
      {
        case 0:
        motorMoveSpeed(-40, -40);
        break;
        case 1:
        motorMoveSpeed(0, 0);
        break;
        case 2:
        motorMoveSpeed(40, 40);
        break;        
      }
      motorMode = (motorMode+1)%3; //0 > 1 > 2 > 0 > 1 > ...      
    }

    //getTCount();
    //1 sec
    if((newTm - prevPrintTm) >= 1000)
    {
      prevPrintTm = newTm;
      
      Serial.print(motorGetRotationAngle());
      Serial.print("o ");
      Serial.print(motorGetDistanceMiddlePoint());
      Serial.print("mm ");
             
      Serial.print(motorGetRPM(L_MOTOR));
      Serial.print("rpm ");      
      Serial.print(motorGetRPM(R_MOTOR));
      Serial.print("rpm ");
      /*
      Serial.print(motorGetCounter(R_MOTOR));
      Serial.print(" ");
      Serial.print(motorGetSpeed(R_MOTOR));
      Serial.print(" ");
      Serial.print(motorGetStartCounter(R_MOTOR));
      Serial.print(" ");
      Serial.println(motorGetGoalSpeed(R_MOTOR));*/
      Serial.println("");
    }
}
