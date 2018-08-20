#ifndef _BATTERY_H_
#define _BATTERY_H_

#define DEF_BATTERY_PIN    0 //A0
#define DEF_BATTERY_REG_R1 3.3f //REG to VIN
#define DEF_BATTERY_REG_R2 1.2f //REG to GND
#define BATTERY_CHECK_TIME 200 //ms

#define MAX_BATTERY_TABLE_CELLS 4
#define MAX_BATTERY_TABLE 8
#define MAX_BATTERY_BUF 10
#define BATTERY_SKIP_DATA_COUNT 2
#define BATTERY_AVERAGE_COUNT (MAX_BATTERY_BUF-(BATTERY_SKIP_DATA_COUNT*2))
typedef struct {
  float voltage;
  int persent;
}BATTERY_LEVEL_S;

const BATTERY_LEVEL_S sBatteryLookupTable[MAX_BATTERY_TABLE_CELLS][MAX_BATTERY_TABLE] = {
 { //cell 1
  {4.2, 100},
  {3.9,  80},
  {3.85, 60},
  {3.82, 50},
  {3.75, 40},
  {3.70, 20},
  {3.68, 10},
  {3.65,  0}
 },
 { //2s
  {8.4, 100},
  {7.8,  80},
  {7.7, 60},
  {7.6, 50},
  {7.5, 40},
  {7.4, 20},
  {7.35, 10},
  {7.3,  0}
 },
 { //3s
  {12.6, 100},
  {11.7,  80},
  {11.55, 60},
  {11.46, 50},
  {11.25, 40},
  {11.1, 20},
  {11.04, 10},
  {10.95,  0}
 },
 {//4s
  {16.8, 100},
  {15.6,  80},
  {15.4, 60},
  {15.28, 50},
  {15.0, 40},
  {14.8, 20},
  {14.7, 10},
  {14.6,  0}
 },
};
class battery{
private:  
  int batteryPin;
  uint16_t batteryBuf[MAX_BATTERY_BUF];
  uint8_t batteryCounter;
  uint32_t prevCheckTime;
  float battertVal;
  
  // VIN / (R1 + R2) * R2 = ADC Voltage 
  // ADC * (R1 + R2) / R2 = VIN
  float R1; //reg to VIN, default 3.3k
  float R2; //reg to GND default 1.2k

  void calBattery();
public:
  battery(void);
  battery(int pin);
  battery(int pin, float r1, float r2);

  void init(int pin, float r1, float r2);
  uint8_t updateBattery();
  float readBattery();
};
#endif

