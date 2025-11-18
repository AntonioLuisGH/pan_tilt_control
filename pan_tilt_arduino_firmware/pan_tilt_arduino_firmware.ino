/*
 * DYNAMIXEL Pan-Tilt ROS 2 Driver Firmware
 *
 * FINAL HARDWARE SETUP (BASED ON YOUR WORKING EXAMPLE):
 * 1. STACK THE DYNAMIXEL SHIELD directly onto the Arduino Mega.
 * 2. NO JUMPER WIRES are needed.
 * 3. Set the Shield's "Upload Switch" to "DYNAMIXEL".
 * 4. Connect 12V Power to the Shield VM.
 * 5. Connect Mega to PC via USB.
 *
 * This firmware uses the default DynamixelShield constructor,
 * which automatically uses Serial1 (pins 18/19) for servos on a Mega
 * and leaves Serial (USB) free for ROS 2.
 */

#include <DynamixelShield.h>

// --- ROS Serial Config (USB) ---
#define ROS_SERIAL Serial
const unsigned long ROS_BAUD_RATE = 115200;

// --- DYNAMIXEL Config ---
const uint8_t PAN_ID = 1;
const uint8_t TILT_ID = 2;
const float DXL_BAUD_RATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

// --- KEY FIX ---
// Use the default constructor, just like your working example.
// The library will automatically select Serial1 for DYNAMIXELs
// on the Mega.
DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// Global variables for servo positions
int32_t pan_pos_goal = 2048; // Center
int32_t tilt_pos_goal = 2048; // Center

void setup() {
  // Start the serial port to ROS (USB)
  ROS_SERIAL.begin(ROS_BAUD_RATE);

  // Start the serial port to DYNAMIXELs (Serial1)
  dxl.begin(DXL_BAUD_RATE);

  // Set the port protocol
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Continuously try to find servos before proceeding
  while (!pingServos()) {
    ROS_SERIAL.println("Servos not found (check JUMPER WIRING/ID/baud). Retrying...");
    delay(1000);
  }

  // --- DEBUG ---
  ROS_SERIAL.println("Servos PINGED successfully!");

  // Set servos to Position Control Mode
  dxl.torqueOff(PAN_ID);
  dxl.torqueOff(TILT_ID);
  dxl.setOperatingMode(PAN_ID, OP_POSITION);
  dxl.setOperatingMode(TILT_ID, OP_POSITION);
  dxl.torqueOn(PAN_ID);
  dxl.torqueOn(TILT_ID);

  // --- DEBUG ---
  ROS_SERIAL.println("Operating mode set. Torque ON.");

  // Read initial positions to set goals
  pan_pos_goal = dxl.getPresentPosition(PAN_ID);
  tilt_pos_goal = dxl.getPresentPosition(TILT_ID);

  // --- DEBUG ---
  ROS_SERIAL.print("Initial positions read: Pan=");
  ROS_SERIAL.print(pan_pos_goal);
  ROS_SERIAL.print(", Tilt=");
  ROS_SERIAL.println(tilt_pos_goal);

  ROS_SERIAL.println("Pan-Tilt Arduino Driver Initialized. Starting loop...");
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

// Timer for a "heartbeat" message
static unsigned long last_heartbeat_time = 0;


void loop() {
  // 1. Listen for commands from ROS (on Serial)
  if (ROS_SERIAL.available() > 0) {
    char c = ROS_SERIAL.read();
    if (c == '\n') {
      // Print the command we just received from ROS
      ROS_SERIAL.print("Received command: ");
      ROS_SERIAL.println(ros_command);
      processRosCommand(ros_command);
      ros_command = ""; // Clear the buffer
    } else {
      ros_command += c;
    }
  }

  // 2. Publish servo state back to ROS periodically (on Serial)
  static unsigned long last_publish_time = 0;
  unsigned long now = millis();
  if (now - last_publish_time > 50) { // 50ms = 20 Hz
    last_publish_time = now;
    publishServoState();
  }

  // 3. Add a "heartbeat" to know the loop is alive (on Serial)
  if (now - last_heartbeat_time > 1000) { // 1000ms = 1 sec
    last_heartbeat_time = now;
    ROS_SERIAL.println("Loop heartbeat...");
  }
}


void processRosCommand(String command) {
  // Command format:
  // 'P<value>' for Pan (e.g., "P2048")
  // 'T<value>' for Tilt (e.g., "T1024")

  if (command.startsWith("P")) {
    pan_pos_goal = command.substring(1).toInt();
    dxl.setGoalPosition(PAN_ID, pan_pos_goal);

  } else if (command.startsWith("T")) {
    tilt_pos_goal = command.substring(1).toInt();
    dxl.setGoalPosition(TILT_ID, tilt_pos_goal);
  }
}


void publishServoState() {
  // Read present positions from servos (from Serial1)
  int32_t pan_pos_current = dxl.getPresentPosition(PAN_ID);
  int32_t tilt_pos_current = dxl.getPresentPosition(TILT_ID);

  // Send in format: "S_<pan_pos>_<tilt_pos>" (to Serial)
  ROS_SERIAL.print("S_");
  ROS_SERIAL.print(pan_pos_current);
  ROS_SERIAL.print("_");
  ROS_SERIAL.println(tilt_pos_current);
}