#include <inttypes.h>
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Bluetooth.h"
//#include "SoftwareSerial.h"
//#define _USE_DEBUG


#define btSerial Serial2

Bluetooth::Bluetooth()
{
}

Bluetooth::Bluetooth(uint8_t rxPin, uint8_t txPin)
{
}

void Bluetooth::begin(uint32_t baud)
{
  state=0;
  serString="";
  btSerial.begin(baud);
}


int Bluetooth::update(void)
{
    //시리얼 데이터가 있는지 체크
  //없으면 그냥 빠져나감.
  if(btSerial.available() <= 0)
    return 0;

  //데이터 있으면 1바이트 읽어옴.
  char ch = btSerial.read();
  // 변수에 추가함.
  serString += ch;
  //Serial.println(ch);
  // 입력 받은 데이터가 '\n' newline 이면...
  if(ch == '\n')
  {
    state = 0;
    subState = 0;
    //여지까지 입력 받은 데이터가 있는지 체크
    if(serString.length()>0)
    {
      //명령어 시작('$') 위치 찾기
      int startCMD = serString.indexOf("$");

      //명령어 시작위치를 찾았다면
      if(startCMD != -1)
      {
        String serArray[10];
        int tmpcnt=0;
        int idx;

        //명령어 시작 값('$') 제거
        String tmpString=serString.substring(startCMD+1);
        //공백 제거
        tmpString.trim();

        // 데이터 파싱
        while(tmpString.length() > 0)
        {
          // ','를 기준으로 짤라서 serArray에 저장.
          idx = tmpString.indexOf(",");
          if(idx == -1)
          {
            //','없다면 마지막 데이터 저장후 빠져나감.
            serArray[tmpcnt] = tmpString;
            serArray[tmpcnt].trim();
            tmpcnt++;
            break;
          }

          serArray[tmpcnt] = tmpString.substring(0, idx);
          tmpString = tmpString.substring(idx+1);
          tmpString.trim();
          serArray[tmpcnt].trim();

          tmpcnt++;
        }
        //명령 serArray[0]이 'set'이냐, 서보 위치 설정
        if(serArray[0].equalsIgnoreCase("set"))
        {
          //명령 serArray[1] 숫자로 변경  - id
          int sId = serArray[1].toInt();
          //명령 serArray[2] 숫자로 변경  - 위치
          int sPos = serArray[2].toInt();
          state=1;
        }
        //명령 serArray[0]이 'get'이냐, 서보 위치 얻기
        else if(serArray[0].equalsIgnoreCase("get"))
        {
          //명령 serArray[1] 숫자로 변경  - id
          int sId = serArray[1].toInt();
          state=2;
          //btSerial.println(servoPos[sId]);
        }
        else if(serArray[0].equalsIgnoreCase("joy"))
        {
          joyX = serArray[1].toInt();
          joyY = serArray[2].toInt();
          state=5;
        }
        else{
            if(serArray[0].equalsIgnoreCase("up"))
            {
                state=3;
                if(serArray[1].equalsIgnoreCase("d"))
                {
                    subState=1;
                }
                else
                {
                    subState=0;
                }
            }
            else if(serArray[0].equalsIgnoreCase("dn"))
            {
                state=3;
                if(serArray[1].equalsIgnoreCase("d"))
                {
                    subState=2;
                }
                else
                {
                    subState=0;
                }
            }
            else if(serArray[0].equalsIgnoreCase("le"))
            {
                state=3;
                if(serArray[1].equalsIgnoreCase("d"))
                {

                        subState=3;
                }
                else
                {
                        subState=0;
                }
            }
            else if(serArray[0].equalsIgnoreCase("ri"))
            {
                state=3;
                if(serArray[1].equalsIgnoreCase("d"))
                {
                    subState=4;

                }
                else
                {

                    subState=0;
                }
            }
            else if(serArray[0].equalsIgnoreCase("a"))
            {
                state=4;
                if(serArray[1].equalsIgnoreCase("d"))
                {
                    subState=0;
                }
                else
                {
                    subState=1;
                }
            }
            else if(serArray[0].equalsIgnoreCase("b"))
            {
                state=4;
                if(serArray[1].equalsIgnoreCase("d"))
                {
                    subState=0;
                }
                else
                {
                    subState=2;
                }
            }
        }
      }
    }
    else{
        return 0;
    }
    serString = "";
    return 1;
  }
  return 0;
}

int Bluetooth::getData(uint8_t *cmd, uint8_t *sub)
{
    *cmd = state;
    *sub = subState;
    return 0;
}

int Bluetooth::getJoyData(int *x, int *y)
{
    *x = joyX;
    *y = joyY;
    return 0;
}
