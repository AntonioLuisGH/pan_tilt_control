/*
 * DYNAMIXEL Pan-Tilt HARDWARE SERIAL Firmware
 *
 * HARDWARE SETUP:
 * 1. DYNAMIXEL SHIELD stacked on Arduino Mega.
 * 2. Switch: "DYNAMIXEL".
 * 3. Power: 12V to Shield.
 * * COMMUNICATION WIRING (Use Pins 16/17 on the Mega):
 * - USB Adapter TX  -> Mega Pin 17 (RX2)
 * - USB Adapter RX  -> Mega Pin 16 (TX2)
 * - USB Adapter GND -> Mega GND
 */

#include <DynamixelShield.h>

// We use "Serial2" which is physically Pins 16 and 17 on the Mega
#define MONITOR_SERIAL Serial2 
const unsigned long MONITOR_BAUD_RATE = 57600;

const uint8_t PAN_ID = 1;
const uint8_t TILT_ID = 2;
const float DXL_BAUD_RATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

void setup() {
  // Start communication with Laptop (Serial2)
  MONITOR_SERIAL.begin(MONITOR_BAUD_RATE);
  
  // Start communication with Servos (Serial1 - Default)
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println("--- PORT STARTED ON SERIAL2 (Pins 16/17) ---");

  while (!pingServos()) {
    MONITOR_SERIAL.println("Pinging servos...");
    delay(1000);
  }

  MONITOR_SERIAL.println("Servos Connected!");

  // Enable Torque
  dxl.torqueOff(PAN_ID);
  dxl.torqueOff(TILT_ID);
  dxl.setOperatingMode(PAN_ID, OP_POSITION);
  dxl.setOperatingMode(TILT_ID, OP_POSITION);
  dxl.torqueOn(PAN_ID);
  dxl.torqueOn(TILT_ID);

  MONITOR_SERIAL.println("Ready. Type P2000 or T2000.");
}

bool pingServos() {
  return dxl.ping(PAN_ID) && dxl.ping(TILT_ID);
}

String serial_command = "";

void loop() {
  // 1. Read Incoming Commands
  while (MONITOR_SERIAL.available() > 0) {
    char c = MONITOR_SERIAL.read();
    if (c == '\n') {
      serial_command.trim(); // Remove spaces/newlines
      if (serial_command.length() > 0) {
        processSerialCommand(serial_command);
      }
      serial_command = ""; 
    } else {
      serial_command += c;
    }
  }

  // 2. Send Status Updates (every 200ms)
  static unsigned long last_publish_time = 0;
  if (millis() - last_publish_time > 200) {
    last_publish_time = millis();
    publishServoState();
  }
}

void processSerialCommand(String command) {
  // Debug print to confirm we heard you
  MONITOR_SERIAL.print("CMD RECEIVED: "); 
  MONITOR_SERIAL.println(command);

  command.toUpperCase();

  if (command.startsWith("P")) {
    int32_t goal = command.substring(1).toInt();
    // Clamp values to avoid crashes
    if (goal < 0) goal = 0;
    if (goal > 4095) goal = 4095;
    
    dxl.setGoalPosition(PAN_ID, goal);
    MONITOR_SERIAL.print("Moving PAN to ");
    MONITOR_SERIAL.println(goal);
    
  } else if (command.startsWith("T")) {
    int32_t goal = command.substring(1).toInt();
    if (goal < 0) goal = 0;
    if (goal > 4095) goal = 4095;

    dxl.setGoalPosition(TILT_ID, goal);
    MONITOR_SERIAL.print("Moving TILT to ");
    MONITOR_SERIAL.println(goal);
  }
}

void publishServoState() {
  int32_t pan_pos = dxl.getPresentPosition(PAN_ID);
  int32_t tilt_pos = dxl.getPresentPosition(TILT_ID);

  MONITOR_SERIAL.print("S_");
  MONITOR_SERIAL.print(pan_pos);
  MONITOR_SERIAL.print("_");
  MONITOR_SERIAL.println(tilt_pos);
}