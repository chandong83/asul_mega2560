/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "motor.h"

#include "src/PID_v1.h"
#include "src/FlexiTimer2.h"

#define EI_ARDUINO_INTERRUPTED_PIN
#include "src/EnableInterrupt/EnableInterrupt.h"


typedef struct
{
    int8_t enc_dir;
  int32_t  counter;

  int32_t  speed;
  int32_t  start_counter;
  double  distance;

  double   pwm_out;

  uint8_t  enc_pin[2];
  uint8_t  mot_pin[2];
  uint8_t  mot_en_pin;

  int8_t   mot_dir;

  // PID
  double enc_speed;
  double goal_speed;
  double pwm_output;
  PID   *p_pid;
} motor_cfg_t;

#define ENCODER_PPR       7
#define ENCODER_COUNT     2   // A or B
#define DC_MOTOR_RAITO    298
#define DC_MOTOR_RPM      90

motor_cfg_t motor_cfg[2];
double mot_rotation_angle;
 #if  defined(__AVR_ATmega2560__)
void setPwmFrequencyMEGA2560(int pin, int divisor);
#endif
static void motorEncoderLeftISR(void);
static void motorEncoderRightISR(void);
static void motorEncoderUpdate(uint8_t ch, uint8_t flag);
static void motorUpdateISR(void);


void motorBegin(void)
{
  mot_rotation_angle = 0;

  motor_cfg[L_MOTOR].mot_dir    = -1;  // 1 or -1
  motor_cfg[L_MOTOR].enc_pin[0] = A10;  // Interrupt Pin
  motor_cfg[L_MOTOR].enc_pin[1] = 41;
  motor_cfg[L_MOTOR].mot_pin[0] = 23;
  motor_cfg[L_MOTOR].mot_pin[1] = 25;
  motor_cfg[L_MOTOR].mot_en_pin = 11;

  motor_cfg[R_MOTOR].mot_dir    = 1;  // 1 or -1
  motor_cfg[R_MOTOR].enc_pin[0] = A11;  // Interrupt Pin
  motor_cfg[R_MOTOR].enc_pin[1] = 43;
  motor_cfg[R_MOTOR].mot_pin[0] = 27;
  motor_cfg[R_MOTOR].mot_pin[1] = 29;
  motor_cfg[R_MOTOR].mot_en_pin = 12;



  for (int i=0; i<2; i++)
  {
    motor_cfg[i].enc_dir = 0;
    motor_cfg[i].distance = 0;
    motor_cfg[i].counter = 0;
    motor_cfg[i].speed   = 0;
    motor_cfg[i].pwm_out = 0;
    motor_cfg[i].p_pid   = new PID(&motor_cfg[i].enc_speed, &motor_cfg[i].pwm_output, &motor_cfg[i].goal_speed, 0.5, 0.3, 0.3, DIRECT);
    motor_cfg[i].start_counter = 0;

    pinMode(motor_cfg[i].enc_pin[0], INPUT_PULLUP);
    pinMode(motor_cfg[i].enc_pin[1], INPUT_PULLUP);
    pinMode(motor_cfg[i].mot_pin[0], OUTPUT);
    pinMode(motor_cfg[i].mot_pin[1], OUTPUT);
   // setPwmFrequencyMEGA2560(motor_cfg[i].mot_en_pin, 1);
   digitalWrite(motor_cfg[i].mot_pin[0], LOW);
   digitalWrite(motor_cfg[i].mot_pin[1], LOW);
   analogWrite(motor_cfg[i].mot_en_pin, 0);


    motor_cfg[i].p_pid->SetSampleTime(10);
    //motor_cfg[i].p_pid->SetOutputLimits(0, DC_MOTOR_RPM);
    motor_cfg[i].p_pid->SetOutputLimits(0, DC_MOTOR_RPM * (ENCODER_PPR*ENCODER_COUNT));

    motor_cfg[i].p_pid->SetMode(AUTOMATIC);
  }

  enableInterrupt( motor_cfg[L_MOTOR].enc_pin[0], motorEncoderLeftISR, CHANGE);
  enableInterrupt( motor_cfg[R_MOTOR].enc_pin[0], motorEncoderRightISR, CHANGE);

  FlexiTimer2::set(10, motorUpdateISR);
  FlexiTimer2::start();
}

int32_t calculateEncCounter2RPM(int32_t count, int32_t checkTimeMs)
{
  //return ((double)count / (ENCODER_PPR*ENCODER_COUNT) * (1000/checkTimeMs) * 60);/// DC_MOTOR_RAITO);
  return ((double)count * (1000/checkTimeMs) * 60 / DC_MOTOR_RAITO);  
}

double calculateEncCounter2Distance(int32_t count)
{
  double d = (double)TIRE_DIAMETER;
  double r = (double)ENCODER_PPR*ENCODER_COUNT;
  double g = (double)DC_MOTOR_RAITO;
  double tick2Distance = ((d*3.14)/(r*g));
  return ((double)count * tick2Distance);
}


int32_t calculateSpeed2RPM(int32_t speed)
{
    return ((double)speed / (ENCODER_PPR*ENCODER_COUNT));
}
int32_t motorGetRPM(uint8_t ch)
{
  return calculateSpeed2RPM(motor_cfg[ch].speed);
}


int32_t motorGetSpeed(uint8_t ch)
{
  return motor_cfg[ch].speed;
}

int32_t motorGetCounter(uint8_t ch)
{
  return motor_cfg[ch].counter;
}

int32_t motorGetPWM(uint8_t ch)
{
  return motor_cfg[ch].pwm_output;
}

int32_t motorGetStartCounter(uint8_t ch)
{
  return motor_cfg[ch].start_counter;
}

int32_t motorGetGoalSpeed(uint8_t ch)
{
  return (int32_t)motor_cfg[ch].goal_speed;
}

double motorGetDistance(uint8_t ch)
{
  return (int32_t)motor_cfg[ch].distance;
}

double motorGetDistanceMiddlePoint()
{
  return (int32_t)(motor_cfg[L_MOTOR].distance+motor_cfg[R_MOTOR].distance)/2;
}

double motorGetRotationAngle()
{
    double b = (double)DISTANCE_BETWEEN_WHEELS;
    return ((motor_cfg[L_MOTOR].distance-motor_cfg[R_MOTOR].distance)/b)*(180/3.141592);
}

void motorSetSpeed(uint8_t ch, int16_t speed)
{
  motor_cfg[ch].goal_speed = (double)speed;
}

void motorClearDistanceInfo()
{
  for (int i=0; i<2; i++)
  {
    motor_cfg[i].distance = 0;
  }
}

void motorMoveSpeed(int16_t left_speed, int16_t right_speed)
{
  if(left_speed < 0)
  {
    motor_cfg[L_MOTOR].enc_dir = 0;
    motor_cfg[L_MOTOR].mot_dir = 1;
    left_speed = 0 - left_speed;
  }
  else
  {
    motor_cfg[L_MOTOR].enc_dir = 1;
    motor_cfg[L_MOTOR].mot_dir = -1;

  }
  if(right_speed < 0)
  {
     motor_cfg[R_MOTOR].enc_dir = 0;
     motor_cfg[R_MOTOR].mot_dir = -1;
     right_speed = 0 - right_speed;
  }
  else
  {
     motor_cfg[R_MOTOR].enc_dir = 1;
     motor_cfg[R_MOTOR].mot_dir = 1;
  }

  motor_cfg[L_MOTOR].goal_speed = (double)left_speed * (ENCODER_PPR*ENCODER_COUNT);
  motor_cfg[R_MOTOR].goal_speed = (double)right_speed * (ENCODER_PPR*ENCODER_COUNT);
}


void motorSetPwm(uint8_t ch, int16_t pwm_data )
{
  uint16_t pwm_out;
/*
  if (pwm_data >= 0)
  {
    pwm_out = pwm_data;
    analogWrite(motor_cfg[ch].mot_en_pin, pwm_out);
    digitalWrite(motor_cfg[ch].mot_pin[0], LOW);
    digitalWrite(motor_cfg[ch].mot_pin[1], HIGH);
  }
  else
  {
    pwm_out = -pwm_data;
    analogWrite(motor_cfg[ch].mot_en_pin, pwm_out);
    digitalWrite(motor_cfg[ch].mot_pin[0], HIGH);
    digitalWrite(motor_cfg[ch].mot_pin[1], LOW);
  }*/
  if(motor_cfg[ch].mot_dir == -1)
  {
     pwm_out = pwm_data;
    analogWrite(motor_cfg[ch].mot_en_pin, pwm_out);
    digitalWrite(motor_cfg[ch].mot_pin[0], LOW);
    digitalWrite(motor_cfg[ch].mot_pin[1], HIGH);
  }
  else
  {
     pwm_out = pwm_data;
    analogWrite(motor_cfg[ch].mot_en_pin, pwm_out);
    digitalWrite(motor_cfg[ch].mot_pin[0], HIGH);
    digitalWrite(motor_cfg[ch].mot_pin[1], LOW);
  }
}
/* 
uint16_t t_counter[100];
uint8_t t_in=0;
uint8_t t_out=0;

uint8_t getTCount()
{
   if(t_in != t_out)
   {    
     if(t_out == 10)
       Serial.println(t_counter[t_out]);
     t_out++;
     if(t_out >= 100)
       t_out=0;  
   }
   return 0;
}
*/
// Motor Update
//
void motorUpdateISR(void)
{
  int16_t pwmDuty=0;
  double tmp=0;
  int32_t encCount=0;
  for (int i=0; i<2; i++)
  {
    
    encCount = motor_cfg[i].counter - motor_cfg[i].start_counter;
    /*
    t_counter[t_in++] =encCount;
    if(t_in >= 100)
      t_in=0;*/
    motor_cfg[i].start_counter = motor_cfg[i].counter;
    motor_cfg[i].speed = calculateEncCounter2RPM(encCount, 10);
    motor_cfg[i].enc_speed = (double)motor_cfg[i].speed;
    if (motor_cfg[i].p_pid->ComputeISR())
    {
      if (motor_cfg[i].goal_speed == 0)
      {
        motorSetPwm(i, 0);
      }
      else
      {
        pwmDuty = map(motor_cfg[i].pwm_output, 0, DC_MOTOR_RPM* (ENCODER_PPR*ENCODER_COUNT), 0, 255);        
        motorSetPwm(i, pwmDuty);
      }
    }
    tmp = calculateEncCounter2Distance(encCount);
    if(tmp != 0){
        if(motor_cfg[i].enc_dir == 1)
        {
            motor_cfg[i].distance = (motor_cfg[i].distance-1) + tmp;
        }
        else
        {
            motor_cfg[i].distance = (motor_cfg[i].distance+1) - tmp;
        }
    }
  }
  //mot_rotation_angle = (mot_rotation_angle-1)+motorGetRotationAngle();
}

void motorEncoderLeftISR(void)
{
  motorEncoderUpdate(L_MOTOR, 1);
}

void motorEncoderRightISR(void)
{
  motorEncoderUpdate(R_MOTOR, 1);
}

void motorEncoderUpdate(uint8_t ch, uint8_t flag)
{
  uint8_t enc_bit = 0^flag;

  if (digitalRead(motor_cfg[ch].enc_pin[0]) == digitalRead(motor_cfg[ch].enc_pin[1]))
  {
    enc_bit = 1^flag;
  }

  switch(enc_bit)
  {
    case 0x01:
    case 0x02:
    motor_cfg[ch].counter += motor_cfg[ch].mot_dir;

      break;

    default:
      motor_cfg[ch].counter -= motor_cfg[ch].mot_dir;
      break;
  }
}


/*
 * http://www.impulseadventure.com/elec/robot-differential-steering.html
 */
void moveMotor(int xAxis, int yAxis, uint8_t speed)
{
  int nJoyX;
  int nJoyY;
  int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

  // TEMP VARIABLES
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

  float fPivYLimit = 32.0;
  Serial.print(xAxis);
  Serial.print(" ");
  Serial.println(yAxis);

  nJoyX = xAxis;
  nJoyY = yAxis;
  Serial.print(xAxis);
  Serial.print(" ");
  Serial.println(yAxis);


  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY/128.0;
  nMotPremixR = nMotPremixR * nJoyY/128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
  nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);




  if(nMotMixL < 0)
  {
    //back(CH1, map(abs(nMotMixL), 0, 127, 0, 255));
    nMotMixL = map(nMotMixL, 0, -127, 0, -90);
  }
  else
  {
    //go(CH1, map(abs(nMotMixL), 0, 127, 0, 255));
    nMotMixL = map(nMotMixL, 0, 127, 0, 90);

  }
  if(nMotMixR < 0)
  {
    //back(CH2, map(abs(nMotMixR), 0, 127, 0, 255));
    nMotMixR = map(nMotMixR, 0, -127, 0, -90);
  }
  else
  {
    nMotMixR = map(nMotMixR, 0, 127, 0, 90);
    //go(CH2, map(abs(nMotMixR), 0, 127, 0, 255));

  }
  Serial.print(nMotMixL);
  Serial.print(",");
  Serial.println(nMotMixR);
  motorMoveSpeed(nMotMixL, nMotMixR);
}

 #if  0// defined(__AVR_ATmega2560__)
void setPwmFrequencyMEGA2560(int pin, int divisor) {
  byte mode;
      switch(divisor) {
      case 1: mode = 0x01; break;
      case 2: mode = 0x02; break;
      case 3: mode = 0x03; break;
      case 4: mode = 0x04; break;
      case 5: mode = 0x05; break;
      case 6: mode = 0x06; break;
      case 7: mode = 0x07; break;
      default: return;
      }

        switch(pin) {
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR0B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      default: return;
    }
}
#endif
