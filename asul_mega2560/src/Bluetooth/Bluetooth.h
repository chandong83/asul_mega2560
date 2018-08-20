#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

typedef struct{
  int xAxis;
  int yAxis;
  uint8_t btn;
}JOYSTICK_DATA;

class Bluetooth
{
public:
   Bluetooth();
   Bluetooth(uint8_t rxPin, uint8_t txPin);
   void begin(uint32_t baud);
   int update();
   int getData(uint8_t *cmd, uint8_t *sub);
   int getJoyData(int *x, int *y);
private:
  String serString;
  int state;
  int subState;
  int joyX;
  int joyY;
};

#endif
