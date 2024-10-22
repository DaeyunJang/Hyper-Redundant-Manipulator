#include <SPI.h>              //Library for using SPI Communication 
#include <mcp2515.h>          //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)

struct can_frame canMsg;
MCP2515 mcp2515(10);                 // SPI CS Pin 10

 
// char cSTX = 0x02;
// char cETX = 0x03;
char cSTX = '/';
char cETX = ';';

void setup()
{
  Serial.begin(115200);                //Begins Serial Communication at 115200 baudrate
  SPI.begin();                       //Begins SPI communication
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
  Serial.println("start can communcation.");
}
 
long fx;
long fy;
long fz;

long tx;
long ty;
long tz;

void loop()
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
//    Serial.print(canMsg.can_id, HEX); // print ID --> A 가 force, B가 torque 아닐까
//    Serial.print(" "); 
//    Serial.print(canMsg.can_dlc, HEX); // print DLC --> 총 길이 말하는듯 지금은 8
//    Serial.print(" ");
//    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
//      Serial.print(canMsg.data[i],DEC); // --> 전체 raw data 출력 코드
//      Serial.print(" ");
//    }
//    Serial.println(" ");

    if (canMsg.can_id == 26){
      // mN 으로 force 계산
      fx = (canMsg.data[0]*256 + canMsg.data[1])-30000;
      fy = (canMsg.data[2]*256 + canMsg.data[3])-30000;
      fz = (canMsg.data[4]*256 + canMsg.data[5])-30000;
    }
    else if (canMsg.can_id == 27){
      // mNm 으로 torque 계산
      tx = (canMsg.data[0]*256 + canMsg.data[1])-30000;
      ty = (canMsg.data[2]*256 + canMsg.data[3])-30000;
      tz = (canMsg.data[4]*256 + canMsg.data[5])-30000;
      // 번갈아서 받기 때문에 한번만 출력
      Serial.write(cSTX);
      Serial.print(fx);
      Serial.print(",");
      Serial.print(fy);
      Serial.print(",");
      Serial.print(fz);
      Serial.print(",");
      Serial.print(tx);
      Serial.print(",");
      Serial.print(ty);
      Serial.print(",");
      Serial.print(tz);
      Serial.write(cETX);
      Serial.println(" ");
    }
  }
}