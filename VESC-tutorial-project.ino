/*
  Copyright 2016-2017 Nathan Kau nathankau@stanford.edu

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <FlexCAN.h>
#include "AngularPDController.h"
#include "buffer.h"
#include "utils.h"
#include "VESC.h"

/******** GLOBAL VARISBLES *********/

// Create CAN object:
// 1st param: 500kbps
// 2nd param: Use CAN0
// 3rd and 4th params: Use alt tx and alt rx pins
FlexCAN CANTransceiver(500000,0,1,1);
static CAN_message_t msg;

// Variable to keep track of the last time a debugging print message was sent
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMillis last_print_shit;

// Variable to keep track of time since command to RM was sent
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMicros vesc1_time_since_command = 0;

// Variable to keep track of time since command to LM was sent
// The elapsedMicros type automatically increments itself every loop execution!
bool vesc2_command_sent = false;


// VESC motor objects

// BOOM SIDE
VESC vesc1(CANTransceiver); // CAN flexcan


// NON-BOOM SIDE
VESC vesc2(CANTransceiver); // CAN flexcan

/****************************/



/********* CONSTANTS *********/

// Send position commands at 200hz (1s / 5000us)
const int UPDATE_PERIOD =  5000; // us

// built-in led pin
int led = 13;

const float MAX_CURRENT = 8.0; // 30 amps seems the max

const int8_t VESC1_CHANNEL_ID = 0;
const int8_t VESC2_CHANNEL_ID = 1;

const float VESC1_OFFSET = -108; // 108
const int VESC1_DIRECTION = -1;

const float VESC2_OFFSET = -210.0;
const int VESC2_DIRECTION = 1;

/****************************/


/**
 * Check for and read motor angles over CAN. 
 * Returns true and stores value if message received, otherwise return false
 * TODO: Test how clearing or not clearing the message buffer affects stability
 **/
bool readAngleOverCAN(FlexCAN& CANrx, float& last_angle_received, int& transmitter_ID) {
  // keep track if a value was received
  bool read = false;

  // parse all available can messages
  while( CANrx.read(msg)) {
    read = true;

    // Very useful, this is the ID encoded by the transitting CAN device
    transmitter_ID = msg.id & 0xFF;

    // toggle LEDs
    // digitalWrite(led, !digitalRead(led));

    // dummy variable for buffer get functino
    int32_t index=0;

    // parse a 32bit float from the can message data buffer
    last_angle_received = buffer_get_float32(msg.buf, 100000, &index);
    return read;
  }
  return read;
}

/**
 * Call this function in the main loop if you want to see the normalized motor angles
 */
void print_shit() {
  if(last_print_shit > 10) {
    last_print_shit -= 10;

    // Serial.println(loop_time);
    Serial.print(vesc1.read());
    Serial.print("\t");
    Serial.println(vesc2.read());
  }
}

/**
 * Process serial commands send from a computer
 */
void process_serial() {
  if(Serial.available()) {
    char c = Serial.read();

    // Process each different command
    switch(c) {
      // TODO : update how position commands are set
      case 'a':
        // PUT YOUR OWN COMMANDS HERE. 
        // FYI if the main loop is sending position commands then
        // any command sent here will be overridden. 
        // EXAMPLE:
        /*
         * vesc1.write(0.0);
         */
      
      default:
        break;
    }
    
    // Clear the buffer after the first byte is read. 
    // So don't send multiple commands at once and expect any but the first to be executed
    Serial.clear();
  }
}

/**
 * Handle any position messages from VESCs
 */
void process_CAN_messages() {
  float last_read_angle;
  int transmitter_ID;

  // time to read angle over can is 9 us
  if(readAngleOverCAN(CANTransceiver, last_read_angle, transmitter_ID)) {
    switch(transmitter_ID) {
      case VESC2_CHANNEL_ID:
        vesc2.update_angle(last_read_angle);
        break;
      case VESC1_CHANNEL_ID:
        vesc1.update_angle(last_read_angle);
        break;
    }
  }
}

void setup() {
  // Initialize CAN bus
  CANTransceiver.begin();
  Serial.println("CAN Transmitter Initialized");

  // Initialize "power on" led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  // Wait for shit to get set up
  delay(1000);
  
  // Initialize VESC controller objects
  vesc1.attach(VESC1_CHANNEL_ID,
                  VESC1_OFFSET,
                  VESC1_DIRECTION,
                  MAX_CURRENT);

  vesc2.attach(VESC2_CHANNEL_ID,
                    VESC2_OFFSET,
                    VESC2_DIRECTION,
                    MAX_CURRENT);
  Serial.begin(115200);

}

void loop() {
    // Process any CAN messages from the motor controllers
    process_CAN_messages();
    
    // IMPORTANT: read any commands sent by the computer
    process_serial();


    // IMPORTANT: for some reason the code doesn't work without this function call
    // Probably has to do with a delay thing
    print_shit();

    /****** Send current messages to VESCs *******/
    // Send position current commands at 200khz aka 5000 us per loop
    // vesc2 command should be sent halfway between vesc1 commands
    
    if(vesc1_time_since_command > UPDATE_PERIOD) {
      vesc1_time_since_command = 0;

      // Start of a new cycle, vesc2 command should be sent after PID_PERIOD/2 us
      vesc2_command_sent = false;

      // Set the PID constants and position
      // This particular set of arguments sets Kd to 0.05, Ki to 0, Kd to 0.0005, and target position to 0 degrees
      vesc2.write_pos_and_pid_gains(0.03, 0, 0.0005, 0.0);
    }

    // This should execute halfway between every RM current command
    if(vesc1_time_since_command > UPDATE_PERIOD/2 && !vesc2_command_sent) {
      vesc2_command_sent = true;

      // Set the PID constants and position
      // This particular set of arguments sets Kd to 0.05, Ki to 0, and Kd to 0.0005, and target position to 0 degrees


      float angle_2 = sin((float)millis()/1000.0)*50.0 + 50.0;
      Serial.println(angle_2);
      vesc1.write_pos_and_pid_gains(0.03, 0, 0.0005, angle_2);
    }
    /****** End of sending current messages to VESCs *******/

}

