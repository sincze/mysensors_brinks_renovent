/**
 * Last Modified 27-09-2017 - S.Incze
 * Reason: Due due NO / NC of Brinks installation
 * Brinks Switches due to a link between 2 points. If Normally Closed is Used and Mysensors disconnects the WTW turns on.
 * That is why Layout changed to connect brinks to NO pins.
 * 
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 */

// Enable debug prints to serial monitor
#define MY_DEBUG                        // 27-09-2017 No Debug Data needed if not connected.

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Assign a specific Node ID to this device
#define MY_NODE_ID 25

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE                 // 27-09-2017 Enabled repeater functionality

// Flash leds on rx/tx/err

// Set blinking period (in milliseconds)
#define MY_DEFAULT_LED_BLINK_PERIOD 300

//#define MY_DEFAULT_ERR_LED_PIN 4
#define MY_DEFAULT_TX_LED_PIN 6               // I am using LED to display the activity TX on D5
#define MY_DEFAULT_RX_LED_PIN 7              // I am using LED to display the activity RX on D6

#define SKETCH_NAME        "Brinks WTW / OTGW"
#define SKETCH_VERSION     "1.2"


#include <MySensors.h>

#define RELAY_1  3  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 3 // Total number of attached relays
#define RELAY_ON 0  // GPIO value to write to turn on attached relay 27-09-2017 Changed due to NO behaviour of the Brinks installation, Normally RELAY_ON 1
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay 27-09-2017 Changed due to NO behaviour of the Brinks installation RELAY_OFF 0


void before()
{ 
  for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    // Set relay to last known state (using eeprom storage)
    digitalWrite(pin, loadState(sensor)?RELAY_ON:RELAY_OFF);
  }
  
}

void setup()
{

}

void presentation()
{
   
  Serial.println(F("-- Init MySensors Brinks WTW"));
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  Serial.print(F("NodeID: "));
  Serial.println(getNodeId());

  for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_BINARY);
  }
}


void loop() // Added 5 minute heartbeat 07-11-2017
{
 const unsigned long fiveMinutes = 5 * 60 * 1000UL;
 static unsigned long lastSampleTime = 0 - fiveMinutes;  // initialize such that a reading is due the first time through loop()

 unsigned long now = millis();
 if (now - lastSampleTime >= fiveMinutes)
 {
    lastSampleTime += fiveMinutes;
    sendHeartbeat;
 }
 // add code to do other stuff here
}

void receive(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS) {
    // Change relay state
    digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
    // Store state in eeprom
    saveState(message.sensor, message.getBool());
    // Write some debug info
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
  }
}

