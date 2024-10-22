
/******************************************************************************************
 * Project : gripper_ws :: main
 *
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/SIFM
 * @version 0.0
 * @date 2024-04-09
 * @copyright Copyright (c) 2024
 * @brief
 * Read Loadcell(HX711) and Force-torque Sensor(CAN comm)
 * Write Serial print
 * @note
 * MCU          : Arduin DUE
 ******************************************************************************************/

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <SPI.h>
#include <mcp2515.h> // mcp2515 라이브러리를 설치해야 합니다. (https://github.com/autowp/arduino-mcp2515/)
#include <HX711.h>

#define FORCE_CAN_ID 0x2A
#define TORQUE_CAN_ID 0x2B
// #define LOADCELL_1_/ID 0x07F

// Load Cell pin
#define NUMBER_OF_LOADCELL_MODULE        2

#define HX711_DOUT_1                     4  // mcu > HX711 no 1 dout pin
#define HX711_SCK_1                      5  // mcu > HX711 no 1 sck pin
#define CALIBRATION_VALUE_1              2000

#define HX711_DOUT_2                     6  // mcu > HX711 no 2 dout pin
#define HX711_SCK_2                      7  // mcu > HX711 no 2 sck pin
#define CALIBRATION_VALUE_2              3000

// CAN comm pin
#define MCP2515_SPI_CS                   10

// Serial communication Parameters
#define BAUDRATE                         115200
#define BANDWIDTH                        10

// ROTS Threads Frequency Setting
#define RTOS_FREQUENCY                   100   // Hz (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE    100   // Hz (Default)
#define RTOS_FREQUENCY_FORCETORQUESENSOR 200   // Hz (Default)
#define RTOS_FREQUENCY_EEPROM_SAVE       250   // H-z (Default)
#define RTOS_FREQUENCY_MONITORING        25    // Hz (Default)
#define RTOS_FREQUENCY_SERIALREADING     500   // Hz (Default)
#define RTOS_FREQUENCY_SERIALWRITING     200   // Hz (Default)
#define TOTAL_SYSTEM_DELAY               1     // ms

#define PIN_BOARD_LED                    LED_BUILTIN    // pin 13 for LED in board


// MCP2515 mcp2515(MCP2515_SPI_CS);                 // SPI CS Pin 10
// SemaphoreHandle_t mutex;
MCP2515 mcp2515(MCP2515_SPI_CS);  // SPI CS Pin 10
struct can_frame kcan_read_msg_;
struct can_frame kcan_send_msg_;
QueueHandle_t integerQueue;

/******************************************************************************************
 * Class & Enum & Structure
 ******************************************************************************************/
//===========================================================
// timer structure
//===========================================================
struct LoopTimeChecker
{
  float loop_time_checker_LoadCellUpdate = -1;
  float loop_time_checker_ForceTorqueUpdate = -1;
  float loop_time_checker_MotorOperation = -1;
  float loop_time_checker_CANReead       = -1;
  float loop_time_checker_EEPROMUpdate   = -1;
  float loop_time_checker_SerialWriting  = -1;
  float loop_time_checker_SerialReading  = -1;
};

/******************************************************************************************
 * Global variables
 ******************************************************************************************/
LoopTimeChecker TimeChecker;
static volatile long fx_ = 0, fy_ = 0, fz_ = 0; 
static volatile long tx_ = 0, ty_ = 0, tz_ = 0;
static volatile long lc_data_[NUMBER_OF_LOADCELL_MODULE] = {0};

// ForceTorqueSensor FTS;
// Loadcell LC;

/******************************************************************************************
 * Functions
 ******************************************************************************************/
uint8_t init_objects();
void PrintOnMutex(String msg);

void FTSNode();
void LCNode();
void SerialWriteNode();
void SerialReadNode();

//===========================================================
// Node Manager (RTOS)
//===========================================================




void SerialWriteNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;
  static unsigned long count_ = 0;

  const char C_STX = '/';
  const char C_ETX = ';';
  String str_send_buffer = "";
  static long count = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;
    str_send_buffer = "";
    str_send_buffer += String(C_STX);
    str_send_buffer += String(fx_) + ",";
    str_send_buffer += String(fy_) + ",";
    str_send_buffer += String(fz_) + ",";
    str_send_buffer += String(tx_) + ",";
    str_send_buffer += String(ty_) + ",";
    str_send_buffer += String(tz_) + ",";
    str_send_buffer += String(lc_data_[0]) + ",";
    str_send_buffer += String(lc_data_[1]);
    str_send_buffer += String(C_ETX);
    // str_send_buffer += String(count++);
    Serial.println(str_send_buffer);
    TimeChecker.loop_time_checker_SerialWriting = temp_time;
    // vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALWRITING) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}

void LCNode(void *pvParameters){

  HX711 lc1, lc2;
  lc1.begin(HX711_DOUT_1, HX711_SCK_1);
  lc1.set_scale(CALIBRATION_VALUE_1);
  lc1.tare();
  lc2.begin(HX711_DOUT_2, HX711_SCK_2);
  lc2.set_scale(CALIBRATION_VALUE_2);
  lc2.tare();

  while(true)
  {
    lc_data_[0] = lc1.read();
    lc_data_[1] = lc2.read();
    // lc_data_[0] = lc1.read() / CALIBRATION_VALUE_1;
    // lc_data_[1] = lc2.read() / CALIBRATION_VALUE_2;
    // lc_data_[0] = lc1.get_units();
    // lc_data_[1] = lc2.get_units();
    // lc_data_[0] = lc1.get_values();
    // lc_data_[1] = lc2.get_values();
  }
  // FTS.spin(pvParameters);
}

void FTSNode(void *pvParameters){

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
  canid_t can_id_force_ = FORCE_CAN_ID;
  canid_t can_id_torque_ = TORQUE_CAN_ID;

  while(true)
  {
    if (mcp2515.readMessage(&kcan_read_msg_) == MCP2515::ERROR_OK) {
      // CAN 메시지 출력
//      Serial.print(kcan_read_msg_.can_id, HEX); // ID 출력
//      Serial.print(" ");
//      Serial.print(kcan_read_msg_.can_dlc, HEX); // DLC 출력
//      Serial.print(" ");
//      for (int i = 0; i < kcan_read_msg_.can_dlc; i++) {
//        Serial.print(kcan_read_msg_.data[i], HEX);
//      }
//      Serial.println("");

      if (kcan_read_msg_.can_id == can_id_force_){  // 26
        // mN 으로 force 계산
        fx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        fy_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        fz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("FORCE");
      }
      else if (kcan_read_msg_.can_id == can_id_torque_){ //27
        // mNm 으로 torque 계산
        tx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        ty_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        tz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("TORQUE");
      }
    }
  }
  // FTS.spin(pvParameters);
}

void SensorsNode(void *pvParameters){

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();                  //Sets CAN at normal mode
  canid_t can_id_force_ = FORCE_CAN_ID;
  canid_t can_id_torque_ = TORQUE_CAN_ID;

  HX711 lc1, lc2;
  lc1.begin(HX711_DOUT_1, HX711_SCK_1);
  lc1.set_scale(CALIBRATION_VALUE_1);
  lc1.tare();
  lc2.begin(HX711_DOUT_2, HX711_SCK_2);
  lc2.set_scale(CALIBRATION_VALUE_2);
  lc2.tare();

  while(true)
  {
    if (mcp2515.readMessage(&kcan_read_msg_) == MCP2515::ERROR_OK) {
      // CAN 메시지 출력
//      Serial.print(kcan_read_msg_.can_id, HEX); // ID 출력
//      Serial.print(" ");
//      Serial.print(kcan_read_msg_.can_dlc, HEX); // DLC 출력
//      Serial.print(" ");
//      for (int i = 0; i < kcan_read_msg_.can_dlc; i++) {
//        Serial.print(kcan_read_msg_.data[i], HEX);
//      }
//      Serial.println("");

      if (kcan_read_msg_.can_id == can_id_force_){  // 26
        // mN 으로 force 계산
        fx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        fy_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        fz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("FORCE");
      }
      else if (kcan_read_msg_.can_id == can_id_torque_){ //27
        // mNm 으로 torque 계산
        tx_ = (kcan_read_msg_.data[0]*256 + kcan_read_msg_.data[1])-30000;
        ty_ = (kcan_read_msg_.data[2]*256 + kcan_read_msg_.data[3])-30000;
        tz_ = (kcan_read_msg_.data[4]*256 + kcan_read_msg_.data[5])-30000;
        // Serial.println("TORQUE");
      }
    }
    lc_data_[0] = lc1.read();
    lc_data_[1] = lc2.read();
  }
  // FTS.spin(pvParameters);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin();

  integerQueue = xQueueCreate(10, // Queue length
                              sizeof(int) // Queue item size
                              );

  if (integerQueue != NULL) {
    xTaskCreate(SerialWriteNode,
                "SerialWriteNode",
                256,
                NULL,
                2,
                NULL);
//    xTaskCreate(SensorsNode,
//                "SensorsNode",
//                256,
//                NULL,
//                2,
//                NULL);
    xTaskCreate(FTSNode,
                "FTSNode",
                128,
                NULL,
                2,
                NULL);
    xTaskCreate(LCNode,
                "LoadcellNode",
                256,
                NULL,
                2,
                NULL);
                
    
  }
}

void loop() {

}
