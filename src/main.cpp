#include <Arduino.h>
#include <DynamixelShield.h>
#include <SoftwareSerial.h>

#define DXL_SERIAL Serial
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 1.0; // AX12A
const uint8_t DXL_ID_CNT = 4;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {5, 6, 7, 8};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Refer to: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-ram-area
// Starting address of the Data to write; Goal Position = 30 for AX12A
const uint16_t SW_START_ADDR = 30;
// Length of the Data to write; Length of Position data of AX12A is 2 bytes
const uint16_t SW_ADDR_LEN = 2;

typedef struct sw_data
{
  int16_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

DynamixelShield dxl;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int16_t goal_position[2] = {250, 650}; // AX12A rotates between positions 250 and 550

void setup()
{
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000); // AX12A set to 1 Mbps
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  for (uint16_t i = 0; i < DXL_ID_CNT; i++)
  {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.is_info_changed = true;

  for (uint16_t i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

uint8_t j = 0;

void loop()
{
  static uint32_t try_count = 0;

  // Insert a new Goal Position to the SyncWrite Packet
  for (uint16_t i = 0; i < DXL_ID_CNT; i++)
  {
    sw_data[i].goal_position = goal_position[j];
  }

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for (uint16_t i = 0; i < sw_infos.xel_count; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");
      DEBUG_SERIAL.println(sw_data[i].goal_position);
    }
    if (j == 0)
    {
      j = 1;
    }
    else
    {
      j = 0;
    }
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(750);
}