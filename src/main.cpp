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

static const uint8_t MOVEMENT_MOTORS_COUNT = 4;
static const uint8_t MOVEMENT_MOTORS_ID_LIST[MOVEMENT_MOTORS_COUNT] = {
    6, // FL
    5, // FR
    8, // RL
    7, // RR
};

static const uint8_t ARM_MOTORS_COUNT = 3;
static const uint8_t ARM_MOTORS_ID_LIST[ARM_MOTORS_COUNT] = {
    9,  // center
    10, // right
    11, // left
};

// Refer to: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-ram-area
// Starting address of the Data to write; Moving Speed = 32 for AX12A
static const uint16_t MOVING_SPEED_START_ADDR = 32;
// Length of the Data to write; Length of Speed data of AX12A is 2 bytes
static const uint16_t MOVING_SPEED_ADDR_LEN = 2;

// Starting address of the Data to write; Goal Position = 30 for AX12A
static const uint16_t GOAL_POSITION_START_ADDR = 30;
// Length of the Data to write; Length of Position data of AX12A is 2 bytes
static const uint16_t GOAL_POSITION_ADDR_LEN = 2;

typedef struct sw_data_speed {
  int16_t speed_goal;
} __attribute__((packed)) sw_data_speed_t;

sw_data_speed_t sw_data_speed[MOVEMENT_MOTORS_COUNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos_speed;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_speed[MOVEMENT_MOTORS_COUNT];

typedef struct sw_data_position {
  int16_t position_goal;
} __attribute__((packed)) sw_data_position_t;

sw_data_position_t sw_data_position[ARM_MOTORS_COUNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos_position;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_position[ARM_MOTORS_COUNT];

uint16_t movement_values[MOVEMENT_MOTORS_COUNT];
uint16_t position_values[ARM_MOTORS_COUNT];

const uint8_t START_BYTE = 0xFF; // Start byte value
const uint8_t MOVEMENT_MESSAGE_FLAG = 0xE0;
const uint8_t ARM_MESSAGE_FLAG = 0xE1;
const uint8_t ARM_MOVE_MESSAGE_FLAG = 0xA0;
const uint8_t ARM_KEEP_POSE_MESSAGE_FLAG = 0xA1;
const uint8_t PAYLOAD_LEN = 8;
const uint8_t MESSAGE_LEN = PAYLOAD_LEN + 1;

const uint16_t ARM_MOTORS_START_POSITION = 512;

const uint8_t ARM_MOTORS_READ_TIMEOUT = 10;

DynamixelShield dxl;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

void handleMovementMessage() {
  // 4 movement_values * 2 bytes each
  for (uint8_t i = 0; i < MOVEMENT_MOTORS_COUNT; i++) {
    movement_values[i] =
        RASPBERRY_SERIAL.read() | (RASPBERRY_SERIAL.read() << 8); // Combine two bytes into a 16-bit value
  }

  // Insert a new Velocity Goal to the SyncWrite Packet
  for (uint8_t i = 0; i < MOVEMENT_MOTORS_COUNT; i++) {
    sw_data_speed[i].speed_goal = movement_values[i];
  }

  // Update the SyncWrite packet status
  sw_infos_speed.is_info_changed = true;

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  dxl.syncWrite(&sw_infos_speed);

  RASPBERRY_SERIAL.print("FL: ");
  RASPBERRY_SERIAL.print(movement_values[0]);
  RASPBERRY_SERIAL.print(", FR: ");
  RASPBERRY_SERIAL.print(movement_values[1]);
  RASPBERRY_SERIAL.print(", RL: ");
  RASPBERRY_SERIAL.print(movement_values[2]);
  RASPBERRY_SERIAL.print(", RR: ");
  RASPBERRY_SERIAL.println(movement_values[3]);
}

void handleArmMoveMessage() {
  for (uint8_t i = 0; i < ARM_MOTORS_COUNT; i++) {
    position_values[i] =
        RASPBERRY_SERIAL.read() | (RASPBERRY_SERIAL.read() << 8); // Combine two bytes into a 16-bit value
  }
  // read the padding byte
  RASPBERRY_SERIAL.read();

  // Insert a new Velocity Goal to the SyncWrite Packet
  for (uint8_t i = 0; i < ARM_MOTORS_COUNT; i++) {
    sw_data_position[i].position_goal = position_values[i];
  }

  // Update the SyncWrite packet status
  sw_infos_position.is_info_changed = true;

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  dxl.syncWrite(&sw_infos_position);

  RASPBERRY_SERIAL.print("Move: ");
  RASPBERRY_SERIAL.print("C: ");
  RASPBERRY_SERIAL.print(position_values[0]);
  RASPBERRY_SERIAL.print(", R: ");
  RASPBERRY_SERIAL.print(position_values[1]);
  RASPBERRY_SERIAL.print(", L: ");
  RASPBERRY_SERIAL.println(position_values[2]);
}

void handleArmKeepPoseMessage() {
  // discard the payload
  for (uint8_t i = 0; i < ARM_MOTORS_COUNT * 2; i++) {
    RASPBERRY_SERIAL.read();
  }
  RASPBERRY_SERIAL.read();

  for (uint8_t i = 0; i < ARM_MOTORS_COUNT; i++) {
    dxl.read(ARM_MOTORS_ID_LIST[i], GOAL_POSITION_START_ADDR, GOAL_POSITION_ADDR_LEN, (uint8_t *)&position_values[i],
             sizeof(position_values[i]), ARM_MOTORS_READ_TIMEOUT);
    sw_data_position[i].position_goal = position_values[i];
  }

  sw_infos_position.is_info_changed = true;
  dxl.syncWrite(&sw_infos_position);

  RASPBERRY_SERIAL.print("Keep pose: ");
  RASPBERRY_SERIAL.print("C: ");
  RASPBERRY_SERIAL.print(position_values[0]);
  RASPBERRY_SERIAL.print(", R: ");
  RASPBERRY_SERIAL.print(position_values[1]);
  RASPBERRY_SERIAL.print(", L: ");
  RASPBERRY_SERIAL.println(position_values[2]);
}

void handleArmMessage() {
  uint8_t arm_message_type = RASPBERRY_SERIAL.read();

  switch (arm_message_type) {
  case ARM_MOVE_MESSAGE_FLAG:
    handleArmMoveMessage();
    break;
  case ARM_KEEP_POSE_MESSAGE_FLAG:
    handleArmKeepPoseMessage();
    break;
  default:
    RASPBERRY_SERIAL.println("Arm message not recognized");
    break;
  }
}

void setup() {
  RASPBERRY_SERIAL.begin(RASPBERRY_SERIAL_BAUD_RATE);

  dxl.begin(DYNAMIXEL_BAUD_RATE);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  for (uint8_t i = 0; i < MOVEMENT_MOTORS_COUNT; i++) {
    dxl.torqueOff(MOVEMENT_MOTORS_ID_LIST[i]);
    dxl.setOperatingMode(MOVEMENT_MOTORS_ID_LIST[i], OP_VELOCITY);
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

  sw_infos_position.addr = GOAL_POSITION_START_ADDR;
  sw_infos_position.addr_length = GOAL_POSITION_ADDR_LEN;
  sw_infos_position.p_xels = info_xels_sw_position;
  sw_infos_position.xel_count = 0;
  sw_infos_position.packet.p_buf = nullptr;
  sw_infos_position.packet.is_completed = false;
  sw_infos_position.is_info_changed = true;

  for (uint8_t i = 0; i < MOVEMENT_MOTORS_COUNT; i++) {
    info_xels_sw_speed[i].id = MOVEMENT_MOTORS_ID_LIST[i];
    info_xels_sw_speed[i].p_data = (uint8_t *)&sw_data_speed[i].speed_goal;
    sw_infos_speed.xel_count++;
  }

  for (uint8_t i = 0; i < ARM_MOTORS_COUNT; i++) {
    sw_data_position[i].position_goal = ARM_MOTORS_START_POSITION;
    info_xels_sw_position[i].id = ARM_MOTORS_ID_LIST[i];
    info_xels_sw_position[i].p_data = (uint8_t *)&sw_data_position[i].position_goal;
    sw_infos_position.xel_count++;
  }

  dxl.syncWrite(&sw_infos_position);

  delay(1000);
}

void loop() {
  static bool sync = false; // Track synchronization state

  if (RASPBERRY_SERIAL.available()) {
    if (!sync) {
      // Look for the start byte
      if (RASPBERRY_SERIAL.read() == START_BYTE) {
        sync = true;
      }
    } else {
      // Wait until enough bytes are available
      if (RASPBERRY_SERIAL.available() >= MESSAGE_LEN) {
        // First byte after sync is a flag
        uint8_t flag = RASPBERRY_SERIAL.read();

        switch (flag) {
        case MOVEMENT_MESSAGE_FLAG:
          handleMovementMessage();
          break;
        case ARM_MESSAGE_FLAG:
          handleArmMessage();
          break;
        default:
          RASPBERRY_SERIAL.println("Message not recognized");
          break;
        }

        sync = false;
      }
    }
  }
}