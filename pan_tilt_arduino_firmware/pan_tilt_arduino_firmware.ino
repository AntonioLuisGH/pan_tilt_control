/*
 * DYNAMIXEL Pan-Tilt ROS 2 Driver Firmware
 *
 * This version is based on your working example.
 * It uses the default `DynamixelShield` constructor, which
 * automatically detects the Arduino Mega and uses:
 * - Serial1 (Pins 18/19) for Servos
 * - Serial (USB) for ROS / Debugging
 *
 * HARDWARE:
 * 1. STACK the DYNAMIXEL Shield directly on the Arduino Mega.
 * 2. (No jumper wires are needed for serial)
 * 3. Set the "Upload Switch" to "DYNAMIXEL" (this is good practice).
 * 4. Connect 12V Power to the Shield VM.
 * 5. Connect the Arduino Mega to your PC via USB.
 */

// Use the correct library that you found!
#include <DynamixelShield.h>

// --- DYNAMIXEL Config ---
// We will use the library's defaults.
// On a Mega, this automatically uses Serial1 (pins 18/19)
// and DIR_PIN 2.
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
// Initialize the DynamixelShield library using the
// default constructor, as shown in your working example.
// The library will auto-detect the Mega and use Serial1.
DynamixelShield dxl;

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
  // (from your working example)
  dxl.begin(DXL_BAUD_RATE);

  // Set the port protocol (from your working example)
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Continuously try to find servos before proceeding
  while (!pingServos()) {
    ROS_SERIAL.println("Servos not found (check stacking/ID/baud). Retrying in 1 sec...");
    delay(1000);
  }

  // --- NEW DEBUG ---
  ROS_SERIAL.println("Servos PINGED successfully!");
  // --- END DEBUG ---

  // Set servos to Position Control Mode (from your example)
  dxl.torqueOff(PAN_ID);
  dxl.torqueOff(TILT_ID);
  dxl.setOperatingMode(PAN_ID, OP_POSITION);
  dxl.setOperatingMode(TILT_ID, OP_POSITION);
  dxl.torqueOn(PAN_ID);
  dxl.torqueOn(TILT_ID);

  // --- NEW DEBUG ---
  ROS_SERIAL.println("Operating mode set. Torque ON.");
  // --- END DEBUG ---

  // Read initial positions to set goals
  pan_pos_goal = dxl.getPresentPosition(PAN_ID);
  tilt_pos_goal = dxl.getPresentPosition(TILT_ID);

  // --- NEW DEBUG ---
  ROS_SERIAL.print("Initial positions read: Pan=");
  ROS_SERIAL.print(pan_pos_goal);
  ROS_SERIAL.print(", Tilt=");
  ROS_SERIAL.println(tilt_pos_goal);
  // --- END DEBUG ---

  ROS_SERIAL.println("Pan-Tilt Arduino Driver Initialized.");
}


bool pingServos() {
  // Use ping() from your working example
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

// --- NEW DEBUG ---
// We add a new timer for a "heartbeat" message
static unsigned long last_heartbeat_time = 0;
// --- END DEBUG ---


void loop() {
  // 1. Listen for commands from ROS
  if (ROS_SERIAL.available() > 0) {
    char c = ROS_SERIAL.read();
    if (c == '\n') {
      // --- NEW DEBUG ---
      // Print the command we just received from ROS
      ROS_SERIAL.print("Received command: ");
      ROS_SERIAL.println(ros_command);
      // --- END DEBUG ---
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

  // --- NEW DEBUG ---
  // 3. Add a "heartbeat" to know the loop is alive
  if (now - last_heartbeat_time > 1000) { // 1000ms = 1 sec
    last_heartbeat_time = now;
    ROS_SERIAL.println("Loop heartbeat...");
  }
  // --- END DEBUG ---
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