#include <Arduino.h>
#include <DynamixelShield.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX

#define DXL_SERIAL Serial
#define RASPBERRY_SERIAL soft_serial

static const int32_t RASPBERRY_SERIAL_BAUD_RATE = 9600;

static const float DYNAMIXEL_PROTOCOL_VERSION = 1.0; // AX12A
static const int32_t DYNAMIXEL_BAUD_RATE = 1000000;  // AX12A use 1 Mbps baud rate
static const uint8_t BROADCAST_ID = 254;

static const uint8_t MOTORS_COUNT = 4;
static const uint8_t MOTORS_ID_LIST[MOTORS_COUNT] = {
    6, // FL
    5, // FR
    8, // RL
    7, // RR
};

// Refer to: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-ram-area
// Starting address of the Data to write; Moving Speed = 32 for AX12A
static const uint16_t MOVING_SPEED_START_ADDR = 32;
// Length of the Data to write; Length of Speed data of AX12A is 2 bytes
static const uint16_t MOVING_SPEED_ADDR_LEN = 2;

typedef struct sw_data_speed
{
  int16_t speed_goal;
} __attribute__((packed)) sw_data_speed_t;

sw_data_speed_t sw_data_speed[MOTORS_COUNT];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos_speed;

DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_speed[MOTORS_COUNT];

DynamixelShield dxl;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

uint16_t values[MOTORS_COUNT];
const uint8_t START_BYTE = 0xFF; // Start byte value

void setup()
{
  RASPBERRY_SERIAL.begin(RASPBERRY_SERIAL_BAUD_RATE);

  dxl.begin(DYNAMIXEL_BAUD_RATE);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  for (uint8_t i = 0; i < MOTORS_COUNT; i++)
  {
    dxl.torqueOff(MOTORS_ID_LIST[i]);
    dxl.setOperatingMode(MOTORS_ID_LIST[i], OP_VELOCITY);
  }

  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos_speed.addr = MOVING_SPEED_START_ADDR;
  sw_infos_speed.addr_length = MOVING_SPEED_ADDR_LEN;
  sw_infos_speed.p_xels = info_xels_sw_speed;
  sw_infos_speed.xel_count = 0;
  sw_infos_speed.packet.p_buf = nullptr;
  sw_infos_speed.packet.is_completed = false;
  sw_infos_speed.is_info_changed = true;

  for (uint8_t i = 0; i < MOTORS_COUNT; i++)
  {
    info_xels_sw_speed[i].id = MOTORS_ID_LIST[i];
    info_xels_sw_speed[i].p_data = (uint8_t *)&sw_data_speed[i].speed_goal;
    sw_infos_speed.xel_count++;
  }
}

void loop()
{
  static bool sync = false; // Track synchronization state

  if (RASPBERRY_SERIAL.available())
  {
    if (!sync)
    {
      // Look for the start byte
      if (RASPBERRY_SERIAL.read() == START_BYTE)
      {
        sync = true;
      }
    }
    else
    {
      // Wait until enough bytes are available
      if (RASPBERRY_SERIAL.available() >= 8)
      { // 4 values * 2 bytes each
        for (int i = 0; i < 4; i++)
        {
          values[i] = RASPBERRY_SERIAL.read() | (RASPBERRY_SERIAL.read() << 8); // Combine two bytes into a 16-bit value
        }
        sync = false; // Reset synchronization after reading data

        // Insert a new Velocity Goal to the SyncWrite Packet
        for (uint8_t i = 0; i < MOTORS_COUNT; i++)
        {
          sw_data_speed[i].speed_goal = values[i];
        }

        // Update the SyncWrite packet status
        sw_infos_speed.is_info_changed = true;

        // Build a SyncWrite Packet and transmit to DYNAMIXEL
        dxl.syncWrite(&sw_infos_speed);

        RASPBERRY_SERIAL.print("m0: ");
        RASPBERRY_SERIAL.print(values[0]);
        RASPBERRY_SERIAL.print(", m1: ");
        RASPBERRY_SERIAL.print(values[1]);
        RASPBERRY_SERIAL.print(", m2: ");
        RASPBERRY_SERIAL.print(values[2]);
        RASPBERRY_SERIAL.print(", m3: ");
        RASPBERRY_SERIAL.println(values[3]);
      }
    }
  }
}