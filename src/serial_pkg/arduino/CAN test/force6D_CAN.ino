#include <SPI.h>              //Library for using SPI Communication 
#include <mcp2515.h>          //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)

#define FORCE_ID 0x2A
#define TORQUE_ID 0x2B

struct can_frame canMsg;
MCP2515 mcp2515(10);                 // SPI CS Pin 10

void parse_fts(can_frame canMsg);
void sendSerial();

// force-torque sensor parameters
struct forcetorque_frame {
  canid_t force_can_id = 0x1A;
  canid_t torque_can_id = 0x1B;
  long fx = 0;
  long fy = 0;
  long fz = 0;
  long tx = 0;
  long ty = 0;
  long tz = 0;
};

char cSTX = '/';
char cETX = ';';

long i;
struct forcetorque_frame ftMsg;
struct loadcell_frame loadcell_frame_1;
struct loadcell_frame loadcell_frame_2;

void setup()
{
  Serial.begin(115200);                //Begins Serial Communication at 115200 baudrate
  SPI.begin();                       //Begins SPI communication
  // struct can_frame cansendMsg;
  // cansendMsg.can_id = 0x0707;
  // cansendMsg.can_dlc = 8;
  // cansendMsg.data[0]=0x81;
  // cansendMsg.data[1]=2;
  // cansendMsg.data[2]=0x03;
  // cansendMsg.data[3]=0xE8;
  ftMsg.force_can_id = FORCE_ID;
  ftMsg.torque_can_id = TORQUE_ID;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
  Serial.println("start can communcation.");
}


void loop()
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
  {
    parse_fts(canMsg);
  }
}

void parse_fts(can_frame canMsg)
{
  if (canMsg.can_id == FORCE_ID){  // 26
    // mN 으로 force 계산
    ftMsg.fx = (canMsg.data[0]*256 + canMsg.data[1])-30000;
    ftMsg.fy = (canMsg.data[2]*256 + canMsg.data[3])-30000;
    ftMsg.fz = (canMsg.data[4]*256 + canMsg.data[5])-30000;
    // Serial.println("FORCE");
  }
  else if (canMsg.can_id == TORQUE_ID){ //27
    // mNm 으로 torque 계산
    ftMsg.tx = (canMsg.data[0]*256 + canMsg.data[1])-30000;
    ftMsg.ty = (canMsg.data[2]*256 + canMsg.data[3])-30000;
    ftMsg.tz = (canMsg.data[4]*256 + canMsg.data[5])-30000;
    // Serial.println("TORQUE");
  }
}


void sendSerial(){
    Serial.write(cSTX);
    Serial.print(ftMsg.fx);
    Serial.print(",");
    Serial.print(ftMsg.fy);
    Serial.print(",");
    Serial.print(ftMsg.fz);
    Serial.print(",");
    Serial.print(ftMsg.tx);
    Serial.print(",");
    Serial.print(ftMsg.fy);
    Serial.print(",");
    Serial.print(ftMsg.fz);
    Serial.write(cETX);
    Serial.print(i++);
    Serial.println(" ");
}
