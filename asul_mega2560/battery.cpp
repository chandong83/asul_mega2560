#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "battery.h"



battery::battery(void)
{
  init(DEF_BATTERY_PIN,DEF_BATTERY_REG_R1,DEF_BATTERY_REG_R2);
}
battery::battery(int pin)
{   
  init(pin,DEF_BATTERY_REG_R1,DEF_BATTERY_REG_R2);
}

battery::battery(int pin, float r1, float r2)
{
  init(pin,r1,r2);
}

void battery::init(int pin, float r1, float r2){  
  batteryCounter=0;
  prevCheckTime=0;
  battertVal=0;
  
  batteryPin = pin; 
  R1 = r1;
  R2 = r2;
}


void battery::calBattery(){
  uint32_t sum=0;
  float avg;
  float voltage;
  uint16_t tmp=0;;
  for(int i=0;i<MAX_BATTERY_BUF-1;i++){
    for(int j=i+1;j<MAX_BATTERY_BUF;j++) {
      if(batteryBuf[i] > batteryBuf[j]){
        tmp = batteryBuf[i];
        batteryBuf[i] = batteryBuf[j];
        batteryBuf[j] = tmp;        
      }
    }
  }
#if 0
  for(int i=0;i<MAX_BATTERY_BUF;i++){
    Serial.print(i);
    Serial.print(" ");
    Serial.println(batteryBuf[i]);
  }
#endif   
  for(int i=BATTERY_SKIP_DATA_COUNT;i<MAX_BATTERY_BUF-BATTERY_SKIP_DATA_COUNT;i++){
    sum += batteryBuf[i];
  }
  avg = (float)sum / BATTERY_AVERAGE_COUNT;  
  voltage = avg*0.0049;
  battertVal = voltage*(R1+R2)/R2;
#if 0
  Serial.print(" sum :");
  Serial.print(sum);  
  Serial.print(" avg :");
  Serial.print(avg);  
  Serial.print(" adc to voltage :");
  Serial.print(voltage);    
  Serial.print(" battery :");
  Serial.print(battertVal);
  Serial.println("V");
#endif 
}

uint8_t battery::updateBattery()
{
  uint32_t ms = millis();
  if((ms - prevCheckTime) > BATTERY_CHECK_TIME){
    prevCheckTime = ms;
    batteryBuf[batteryCounter++] = analogRead(batteryPin);
    if(batteryCounter >= MAX_BATTERY_BUF){
      calBattery();
      batteryCounter = 0;
      return 1;
    }
  }
  return 0;
}
float battery::readBattery()
{
  return battertVal;
}
