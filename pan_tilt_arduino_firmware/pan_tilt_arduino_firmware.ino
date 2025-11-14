/*
 * DYNAMIXEL Pan-Tilt ROS 2 Driver Firmware
 *
 * This version uses the <DynamixelShield.h> library, which you
 * proved is the correct one for your setup.
 *
 * It still requires an Arduino Mega and jumper wires to use
 * the separate Serial1 port for servos, which is critical
 * for talking to ROS 2 at the same time.
 *
 * HARDWARE WIRING (Arduino Mega):
 * - Shield Pin 0 (RX)  -> Mega Pin 18 (TX1)
 * - Shield Pin 1 (TX)  -> Mega Pin 19 (RX1)
 * - Shield Pin 2 (DIR) -> Mega Pin 2
 * - Shield GND         -> Mega GND
 * - 12V Power to Shield VM
 * - USB to PC
 */

// Use the correct library that you found!
#include <DynamixelShield.h>

// --- DYNAMIXEL Config ---
// This serial port is for the DYNAMIXELs
// On Mega, Serial1 is pins 18 (TX) and 19 (RX)
#define DXL_SERIAL   Serial1
// const uint8_t DXL_DIR_PIN = 2; // Direction pin <-- DELETE THIS LINE
const uint8_t PAN_ID = 1;
const uint8_t TILT_ID = 2;

// Use the settings from your working example
const float DXL_BAUD_RATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

// --- ROS Serial Config ---
// This serial port is for the PC (ROS 2) over USB
#define ROS_SERIAL Serial
const unsigned long ROS_BAUD_RATE = 115200;

// --- KEY FIX ---
// Initialize the DynamixelShield library, but tell it
// to use our custom Serial1 port and DIR pin.
DynamixelShield dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// Global variables for servo positions
int32_t pan_pos_goal = 2048; // Center
int32_t tilt_pos_goal = 2048; // Center

void setup() {
  // Start the serial port to ROS (USB)
  ROS_SERIAL.begin(ROS_BAUD_RATE);
  // while (!ROS_SERIAL); // Wait for serial to open

  // Start the serial port to DYNAMIXELs (Serial1)
  dxl.begin(DXL_BAUD_RATE);

  // Set the port protocol (from your working example)
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Continuously try to find servos before proceeding
  while (!pingServos()) {
    ROS_SERIAL.println("Servos not found (check wiring/ID/baud). Retrying in 1 sec...");
    delay(1000);
  }

  // Set servos to Position Control Mode (default is 3, which is position)
  dxl.setOperatingMode(PAN_ID, OP_POSITION);
  dxl.setOperatingMode(TILT_ID, OP_POSITION);

  // Turn on servo torque
  dxl.torqueOn(PAN_ID);
  dxl.torqueOn(TILT_ID);

  // Read initial positions to set goals
  pan_pos_goal = dxl.getPresentPosition(PAN_ID);
  tilt_pos_goal = dxl.getPresentPosition(TILT_ID);

  ROS_SERIAL.println("Pan-Tilt Arduino Driver Initialized.");
}


bool pingServos() {
  bool panOk = dxl.ping(PAN_ID);
  if (!panOk) {
    ROS_SERIAL.println("Failed to ping PAN servo (ID 1)");
  }

  bool tiltOk = dxl.ping(TILT_ID);
  if (!tiltOk) {
    ROS_SERIAL.println("Failed to ping TILT servo (ID 2)");
  }

  return panOk && tiltOk;
}


// Buffer for incoming serial data from ROS
String ros_command = "";

void loop() {
  // 1. Listen for commands from ROS
  if (ROS_SERIAL.available() > 0) {
    char c = ROS_SERIAL.read();
    if (c == '\n') {
      processRosCommand(ros_command);
      ros_command = ""; // Clear the buffer
    } else {
      ros_command += c;
    }
  }

  // 2. Publish servo state back to ROS periodically
  static unsigned long last_publish_time = 0;
  unsigned long now = millis();
  if (now - last_publish_time > 50) { // 50ms = 20 Hz
    last_publish_time = now;
    publishServoState();
  }
}


void processRosCommand(String command) {
  // Command format:
  // 'P<value>' for Pan (e.g., "P2048")
  // 'T<value>' for Tilt (e.g., "T1024")

  if (command.startsWith("P")) {
    pan_pos_goal = command.substring(1).toInt();
    dxl.setGoalPosition(PAN_ID, pan_pos_goal);
    // ROS_SERIAL.println("OK PAN"); // Optional: send acknowledgment

  } else if (command.startsWith("T")) {
    tilt_pos_goal = command.substring(1).toInt();
    dxl.setGoalPosition(TILT_ID, tilt_pos_goal);
    // ROS_SERIAL.println("OK TILT"); // Optional: send acknowledgment
  }
}


void publishServoState() {
  // Read present positions from servos
  int32_t pan_pos_current = dxl.getPresentPosition(PAN_ID);
  int32_t tilt_pos_current = dxl.getPresentPosition(TILT_ID);

  // Send in format: "S_<pan_pos>_<tilt_pos>"
  ROS_SERIAL.print("S_");
  ROS_SERIAL.print(pan_pos_current);
  ROS_SERIAL.print("_");
  ROS_SERIAL.println(tilt_pos_current);
}