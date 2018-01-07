
/**
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
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 * 
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 * 
 * Used PINS:
 * 
 * A00,01,04  Relais, analog pins however can be used as digital as well.
 * D03        Temperature Sensor
 * D05,D06    Door / Leakage
 * A05,D08    LED Controls  
 * D08        Filter Warning 
 */

// Enable debug prints to serial monitor
#define MY_DEBUG
boolean debug = false;                                  // Used for additional debugging lines in serial console;

// Enable and select radio type attached
#define MY_RADIO_NRF24                                  // Antenna not used is the: #define MY_RADIO_RFM69

// Assign a specific Node ID to this device
//#define MY_NODE_ID 25

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE                           // 18-11-2017 Enabled repeater functionality

// Flash leds on rx/tx/err
// Set blinking period (in milliseconds)
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Uncomment to override default HW configurations
#define MY_DEFAULT_TX_LED_PIN  2                        // the PCB, on board YELLOW LED
#define MY_DEFAULT_RX_LED_PIN  A5  // Receive led pin  // Receive led pin  GREEN LED

// Household Material to explain which sensor this is
#define SKETCH_NAME        "CV Ruimte Controller"       // Does WTW control (using 2 Relais
#define SKETCH_VERSION     "0.6"                        // Can Reset the OTGW (using 1 relais)
                                                        // Measures Temperature (using 3 sensors
                                                        // Checks for water Leakage under CV using custom mswitch (inverted)
                                                        // Checks the status of the Door to switch on the light ;-)
                                                        // Receives a filter message from BRINKS RENOVENT
// Mysensors From here
#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>                          // Needed for temperature sensor
#include <OneWire.h>                                    // Needed for temperature sensor
#include <Bounce2.h>                                    // Needed for doorsensor

//--------------------------------------------------------------------------------------------
// Relais Part // The following is necessary for the Relais OTGW / WTW
//--------------------------------------------------------------------------------------------

#define NUMBER_OF_RELAYS 3                              // Total number of attached relays
#define RELAY_ON 0                                      // GPIO value to write to turn on attached relay 27-09-2017 Changed due to NO behaviour of the Brinks installation, Normally RELAY_ON 1
#define RELAY_OFF 1                                     // GPIO value to write to turn off attached relay 27-09-2017 Changed due to NO behaviour of the Brinks installation RELAY_OFF 0

const int RELAY[] = {A0, A1, A4};                       // I/O pins for the relays

//--------------------------------------------------------------------------------------------
// Door / Switches (Leakage and Door) // Input for filter detection
//--------------------------------------------------------------------------------------------

#define NUMBER_OF_SWITCHES 3                            // Number of door switches
#define CHILD_ID_DOOR_1 4                               // Door sensors are childs 4 and 5
#define CHILD_ID_DOOR_2 5                               // Door sensors are childs 4 and 5
#define CHILD_ID_DOOR_3 6                               // Door sensors are childs 4 and 5 and 6

Bounce debouncer[NUMBER_OF_SWITCHES];                   // Set up debouncer (used for door sensors)
uint8_t oldValueContact[NUMBER_OF_SWITCHES];            // used to keep track of previous values contact sensor values
byte switchPin[NUMBER_OF_SWITCHES] = {5,6,8};             // what digital pin we're connected to Digital Pin 5 Door
                                                        // what digital pin we're connected to Digital Pin 6 Leakage
                                                        // what digital pin we're connected to Digital Pin 8 Filter Full
MyMessage msgDoor_1(CHILD_ID_DOOR_1, V_TRIPPED);        // Initialize door message for door sensor
MyMessage msgDoor_2(CHILD_ID_DOOR_2, V_TRIPPED);        // Initialize door message for leakage sensor
MyMessage msgDoor_3(CHILD_ID_DOOR_3, V_TRIPPED);        // Initialize door message for Filter Full

//--------------------------------------------------------------------------------------------
// Temperature Part // Necessary for the Dallas Temperature Sensor.
//--------------------------------------------------------------------------------------------

#define COMPARE_TEMP 1                                // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3                                // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
//#define CHILD_ID_TEMP
//#define CHILD_ID_TEMP AUTO                            // Initialize door message, Initialize temperature message or set to AUTO if you want gw to assign a NODE_ID fr you.
//#define CHILD_ID_TEMP1 1
//#define CHILD_ID_TEMP2 2
//#define CHILD_ID_TEMP3 3

unsigned long sleepTimer = 60000;
unsigned long SLEEP_TIME = 60000;                     // Sleep time between reads (in milliseconds)
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;                                     // Number of Temp Sensors
boolean receivedConfig = false;
boolean metric = true; 

OneWire oneWire(ONE_WIRE_BUS);                        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);                  // Pass the oneWire reference to Dallas Temperature. 

//MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);           // Initialize temperature message
MyMessage msgTemp(0, V_TEMP);                         // Initialize temperature message
//MyMessage msgTemp_1(CHILD_ID_TEMP1, V_TEMP);        // Initialize temperature message
//MyMessage msgTemp_2(CHILD_ID_TEMP2, V_TEMP);        // Initialize temperature message
//MyMessage msgTemp_3(CHILD_ID_TEMP3, V_TEMP);        // Initialize temperature message

//--------------------------------------------------------------------------------------------


void before()
{
  //--------------------------------------------------------------------------------------------
  // Startup the Relais Part // The following is necessary for the Relais OTGW / WTW
  //--------------------------------------------------------------------------------------------

  for (int sensor=1, pin=0; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
    pinMode(RELAY[pin], OUTPUT);                                        // Then set relay pins in output mode
    digitalWrite(RELAY[pin], loadState(sensor)?RELAY_ON:RELAY_OFF);     // Set relay to last known state (using eeprom storage)
    if (debug) {
    Serial.print("-- Hi Setting up relays starting at 1 #");
    Serial.println(sensor);
    }
  }

  //--------------------------------------------------------------------------------------------
  // Door / Switches (Leakage and Door) & Filter Warning
  //--------------------------------------------------------------------------------------------

  for (int i = 0; i < NUMBER_OF_SWITCHES; i++) {        // Startup the Debouncer for the door / leakage switch
    Bounce debouncer[i] = Bounce();
    if (debug) {
    Serial.print("-- Hi Setting up debouncers starting at 0 #");
    Serial.println(i);
    }
  }

  //--------------------------------------------------------------------------------------------
  // Temperature Part // Necessary for the Dallas Temperature Sensor.
  //--------------------------------------------------------------------------------------------

  sensors.begin();                                      // Startup up the OneWire library
  
}


void setup() 
{ 
  //--------------------------------------------------------------------------------------------
  // Door / Switches (Leakage and Door) / Input of Filter Warning does not need pull up!
  //--------------------------------------------------------------------------------------------
  
  pinMode(switchPin[0],INPUT_PULLUP);        // D5 Door PIN needs pull up
  pinMode(switchPin[1],INPUT_PULLUP);        // D6 Leakage PIN needs pull up
  pinMode(switchPin[2],INPUT);               // D8 Filter PIN does NOT need pull up!! Danger Danger overvoltage.

  for (int i = 0; i < NUMBER_OF_SWITCHES; i++) {            // Set up door contacts
//  pinMode(switchPin[i], INPUT);
//  digitalWrite(switchPin[i], HIGH);
    oldValueContact[i] = 1;
    debouncer[i].attach(switchPin[i]);
    debouncer[i].interval(5);
  }

  //--------------------------------------------------------------------------------------------
  // Temperature Part // Necessary for the Dallas Temperature Sensor.
  //--------------------------------------------------------------------------------------------

  sensors.setWaitForConversion(false);                     // requestTemperatures() will not block current thread
}


void presentation() 
{
  Serial.println(F("-- Init MySensors CV Ruimte"));
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);              // Send the sketch version information to the gateway and Controller
  Serial.print(F("NodeID: "));
  Serial.println(getNodeId());
  int sensor=1;                                             // This is the first sensor ID we will present to the gateway.

  //--------------------------------------------------------------------------------------------
  // Presentation of the relais to the gateway
  //--------------------------------------------------------------------------------------------

  for (int i=0; i < NUMBER_OF_RELAYS; sensor++, i++) {
    //present(sensor, S_BINARY);                              
    present(sensor, S_BINARY, "Relais", "true");                              // Register all sensors to gw (they will be created as child devices)

    if (debug) {
    Serial.print(F("-- Hi Presenting Relays to gateway starting at 1 #"));
    //Serial.print("-- Hi Presenting Relays to gateway starting at 1 #");
    Serial.println(sensor);
    //Serial.println(F(sensor));
    }
    delay(250);
  }  

  //--------------------------------------------------------------------------------------------
  // Presentation of Door / Switches (Leakage and Door)
  //--------------------------------------------------------------------------------------------

  for (int i=0; i < NUMBER_OF_SWITCHES; sensor++, i++) {
    present(sensor, S_DOOR, "Door/Leakage", "true");                          // Register all switches to gw (they will be created as child devices)
    
    if (debug) {
       Serial.print(F("-- Hi Presenting Door Switches to gateway starting at 4 #"));
       //Serial.print("-- Hi Presenting Door Switches to gateway starting at 4 #");
       Serial.println(sensor);
    }
    delay(250);
  }
  

  
  //--------------------------------------------------------------------------------------------
  // Presentation of the temp sensor
  //--------------------------------------------------------------------------------------------
    numSensors = sensors.getDeviceCount();                                      // Fetch the number of attached temperature sensors  

     if (debug) {
      //Serial.print("-- Hi I could find sensors #");
      Serial.print(F("-- Hi I could find a total of Dallas Temp Sensorss #"));
      Serial.println(numSensors);
     }

  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; sensor++, i++) {        
     present(i+10, S_TEMP, "Temp Sensor", "true");                          // Register all temp sensors to gw (they will be created as child devices)
     
     if (debug) {
        //Serial.print("-- Hi Presenting Temp Sensor to gateway starting at 6 #");
        Serial.print(F("-- Hi Presenting Temp Sensor to gateway starting at 6 #"));
        Serial.println(sensor);
     }
     delay(250);
  }  
}

void loop()     
{     
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)                            // check for switch updates
  {
    debouncer[i].update();
    uint8_t ValueContact = debouncer[i].read();
    if (ValueContact != oldValueContact[i]) 
    {
      //send(msgDoor.setSensor(i).set(ValueContact == HIGH? true : false), false); 
      // 1 Doorsensor is inverted... It is the leakage sensor.  
      if (i==0) {  // Send Specific message for this switch
         //send(msgDoor.setSensor(i).set(ValueContact==HIGH ? 0 : 1));  
         send(msgDoor_1.set(ValueContact==HIGH ? 0 : 1));  
         //send(msgDoor_1.setSensor[i].set(ValueContact==HIGH ? 0 : 1));  
      }

      if (i==1) {   // Send Specific message for this switch
         //send(msgDoor_2.setSensor(i).set(ValueContact==HIGH ? 1 : 0));  
         //send(msgDoor_2.set(ValueContact==HIGH ? 0 : 1));  
         send(msgDoor_2.set(ValueContact==HIGH ? 1 : 0));  
         
      }

      if (i==2) {  // Send Specific message for this switch
      send(msgDoor_3.set(ValueContact==HIGH ? 1 : 0), false);  
      }
      
      if (debug) {
         //Serial.print("Sending doorvalue to controller: ");
         Serial.print(F("Sending doorvalue to controller: "));
         Serial.println(ValueContact);
       } 
    }
    oldValueContact[i] = ValueContact;
  }
  
  if (sleepTimer == SLEEP_TIME)
  {
    // handling of Temperature Sensors  
    sensors.requestTemperatures();                    // Fetch temperatures from Dallas sensors

    // query conversion time and sleep until conversion completed
    int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());

    /*if (debug) {
       Serial.print("sleepTimer: ");
       Serial.println(" %\t");
     }
    */
        
    // Read temperatures and send them to controller 
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
    // Fetch and round temperature to one decimal
      float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
    
      // Only send data if temperature has changed and no error
        #if COMPARE_TEMP == 1
          if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
        #else
          if (temperature != -127.00 && temperature != 85.00) {
        #endif 
          // Send in the new temperature
          send(msgTemp.setSensor(i+10).set(temperature,1));
          // Save new temperatures for next compare
          lastTemperature[i]=temperature;
          }
          sleepTimer = 0;  
        }
     }
     else sleepTimer++;
} // End of Loop


void receive(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS) {
    
    digitalWrite(RELAY[message.sensor-1], message.getBool()?RELAY_ON:RELAY_OFF);  // Change relay state
    saveState(message.sensor, message.getBool());                                 // Store state in eeprom

    if (debug) {
        
        //Serial.print("Incoming change for sensor:");                              // Write some debug info
        Serial.print(F("Incoming change for sensor:"));
        Serial.print(message.sensor);
        //Serial.print(", New status: ");
        Serial.print(F(", New status: "));
        Serial.println(message.getBool());
    }
  }
}

