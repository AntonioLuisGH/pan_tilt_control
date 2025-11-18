/*
 * DYNAMIXEL Pan-Tilt ROS 2 Driver Firmware (Hardware Serial Version)
 *
 * HARDWARE SETUP:
 * 1. DYNAMIXEL SHIELD stacked on Arduino Mega.
 * 2. Switch: "DYNAMIXEL".
 * 3. Power: 12V to Shield.
 * * COMMUNICATION WIRING (Same as your working test):
 * - USB Adapter TX  -> Mega Pin 17 (RX2)
 * - USB Adapter RX  -> Mega Pin 16 (TX2)
 * - USB Adapter GND -> Mega GND
 */

#include <DynamixelShield.h>

// --- ROS Serial Config (Hardware Serial 2) ---
// This uses Pins 16 (TX2) and 17 (RX2)
#define ROS_SERIAL Serial2 
const unsigned long ROS_BAUD_RATE = 57600; // Matched to your working setup

// --- DYNAMIXEL Config ---
const uint8_t PAN_ID = 1;
const uint8_t TILT_ID = 2;
const float DXL_BAUD_RATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

// This namespace is required to use Control table item names
using namespace ControlTableItem;

// Global variables for servo positions
int32_t pan_pos_goal = 2048; 
int32_t tilt_pos_goal = 2048; 

void setup() {
  // Start the serial port to ROS (Via Pins 16/17)
  ROS_SERIAL.begin(ROS_BAUD_RATE);

  // Start the serial port to DYNAMIXELs (Serial1 - Default)
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Continuously try to find servos before proceeding
  while (!pingServos()) {
    ROS_SERIAL.println("Servos not found. Retrying...");
    delay(1000);
  }

  ROS_SERIAL.println("Servos PINGED successfully!");

  // Set servos to Position Control Mode
  dxl.torqueOff(PAN_ID);
  dxl.torqueOff(TILT_ID);
  dxl.setOperatingMode(PAN_ID, OP_POSITION);
  dxl.setOperatingMode(TILT_ID, OP_POSITION);
  dxl.torqueOn(PAN_ID);
  dxl.torqueOn(TILT_ID);

  ROS_SERIAL.println("Operating mode set. Torque ON.");

  // Read initial positions
  pan_pos_goal = dxl.getPresentPosition(PAN_ID);
  tilt_pos_goal = dxl.getPresentPosition(TILT_ID);

  ROS_SERIAL.println("Pan-Tilt Driver Ready on Serial2.");
}


bool pingServos() {
  return dxl.ping(PAN_ID) && dxl.ping(TILT_ID);
}


// Buffer for incoming serial data from ROS
String ros_command = "";

void loop() {
  // 1. Listen for commands from ROS (on Serial2)
  while (ROS_SERIAL.available() > 0) {
    char c = ROS_SERIAL.read();
    if (c == '\n') {
      // Trim whitespace to avoid errors
      ros_command.trim();
      if (ros_command.length() > 0) {
        // Optional: Uncomment to debug, but can slow down ROS
        // ROS_SERIAL.print("Received: ");
        // ROS_SERIAL.println(ros_command);
        processRosCommand(ros_command);
      }
      ros_command = ""; // Clear the buffer
    } else {
      ros_command += c;
    }
  }

  // 2. Publish servo state back to ROS periodically
  static unsigned long last_publish_time = 0;
  unsigned long now = millis();
  
  // Keep this at 200ms (5Hz) initially to ensure stability
  if (now - last_publish_time > 200) { 
    last_publish_time = now;
    publishServoState();
  }
}


void processRosCommand(String command) {
  command.toUpperCase(); // Handle 'p' vs 'P'

  if (command.startsWith("P")) {
    pan_pos_goal = command.substring(1).toInt();
    // Safety clamp
    if(pan_pos_goal < 0) pan_pos_goal = 0;
    if(pan_pos_goal > 4095) pan_pos_goal = 4095;
    
    dxl.setGoalPosition(PAN_ID, pan_pos_goal);

  } else if (command.startsWith("T")) {
    tilt_pos_goal = command.substring(1).toInt();
    // Safety clamp
    if(tilt_pos_goal < 0) tilt_pos_goal = 0;
    if(tilt_pos_goal > 4095) tilt_pos_goal = 4095;
    
    dxl.setGoalPosition(TILT_ID, tilt_pos_goal);
  }
}


void publishServoState() {
  int32_t pan_pos_current = dxl.getPresentPosition(PAN_ID);
  int32_t tilt_pos_current = dxl.getPresentPosition(TILT_ID);

  // Send in format: "S_<pan_pos>_<tilt_pos>"
  ROS_SERIAL.print("S_");
  ROS_SERIAL.print(pan_pos_current);
  ROS_SERIAL.print("_");
  ROS_SERIAL.println(tilt_pos_current);
}