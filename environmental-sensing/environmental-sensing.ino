/*    _   _ _ _____ _    _              _____     _ _     ___ ___  _  __
 *   /_\ | | |_   _| |_ (_)_ _  __ _ __|_   _|_ _| | |__ / __|   \| |/ /
 *  / _ \| | | | | | ' \| | ' \/ _` (_-< | |/ _` | | / / \__ \ |) | ' <
 * /_/ \_\_|_| |_| |_||_|_|_||_\__, /__/ |_|\__,_|_|_\_\ |___/___/|_|\_\
 *                             |___/
 *
 * Copyright 2018 AllThingsTalk
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This experiment shows how LoRa can be used to monitor the quality of
 * your surrounding environment. Measure in- and outdoor air quality, noise
 * levels and temperature to provide stakeholders a dashboard to support
 * their decision making to improve quality of living.
 */
 
// Select your preferred method of sending data
#define CONTAINERS
//#define CBOR
//#define BINARY

/***************************************************************************/



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "AirQuality2.h"
#include <ATT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>

#define SERIAL_BAUD 57600

#define debugSerial Serial
#define loraSerial Serial1

#define AirQualityPin A0
#define LightSensorPin A2
#define SoundSensorPin A4

#define SEND_EVERY 500

MicrochipLoRaModem modem(&loraSerial, &debugSerial);
ATTDevice device(&modem, &debugSerial, false, 7000);  // minimum time between 2 messages set at 7000 milliseconds

#ifdef CONTAINERS
  #include <Container.h>
  Container container(device);
#endif

#ifdef CBOR
  #include <CborBuilder.h>
  CborBuilder payload(device);
#endif

#ifdef BINARY
  #include <PayloadBuilder.h>
  PayloadBuilder payload(device);
#endif

AirQuality2 airqualitysensor;
Adafruit_BME280 tph; // I2C

float soundValue;
float lightValue;
float temp;
float hum;
float pres;
int16_t airValue;

void setup() 
{
  pinMode(GROVEPWR, OUTPUT);  // turn on the power for the secondary row of grove connectors
  digitalWrite(GROVEPWR, HIGH);

  debugSerial.begin(SERIAL_BAUD);
  debugSerial.begin(SERIAL_BAUD);
  while((!debugSerial) && (millis()) < 10000){}  // wait until the serial bus is available

  loraSerial.begin(modem.getDefaultBaudRate());  // set baud rate of the serial connection to match the modem
  while((!loraSerial) && (millis()) < 10000){}   // wait until the serial bus is available

  while(!device.initABP(DEV_ADDR, APPSKEY, NWKSKEY))
  debugSerial.println("Ready to send data");
  
  debugSerial.println();
  debugSerial.println("-- Environmental Sensing LoRa experiment --");
  debugSerial.println();

  initSensors();
}

void loop() 
{
  readSensors();
  displaySensorValues();
  sendSensorValues();
  
  debugSerial.print("Delay for: ");
  debugSerial.println(SEND_EVERY);
  debugSerial.println();
  delay(SEND_EVERY);
}

void initSensors()
{
  debugSerial.println("Initializing sensors, this can take a few seconds...");
  
  pinMode(SoundSensorPin, INPUT);
  pinMode(LightSensorPin, INPUT);
  
  tph.begin();
  airqualitysensor.init(AirQualityPin);
  debugSerial.println("Done");
}

void readSensors()
{
    debugSerial.println("Start reading sensors");
    debugSerial.println("---------------------");
    
    soundValue = analogRead(SoundSensorPin);
    lightValue = analogRead(LightSensorPin);
    lightValue = lightValue * 3.3 / 1023;  // convert to lux based on the voltage that the sensor receives
    lightValue = pow(10, lightValue);
    
    temp = tph.readTemperature();
    hum = tph.readHumidity();
    pres = tph.readPressure()/100.0;
    
    airValue = airqualitysensor.getRawData();
}

void process()
{
  while(device.processQueue() > 0)
  {
    debugSerial.print("QueueCount: ");
    debugSerial.println(device.queueCount());
    delay(100);
  }
}

void sendSensorValues()
{
  #ifdef CONTAINERS
  debugSerial.println("Start sending data to the ATT cloud platform");
  debugSerial.println("--------------------------------------------");
  container.addToQueue(soundValue, LOUDNESS_SENSOR, false); process();
  container.addToQueue(lightValue, LIGHT_SENSOR, false); process();
  container.addToQueue(temp, TEMPERATURE_SENSOR, false); process();
  container.addToQueue(hum, HUMIDITY_SENSOR, false); process();
  container.addToQueue(pres, PRESSURE_SENSOR, false); process();
  container.addToQueue(airValue, AIR_QUALITY_SENSOR, false); process();
  #endif

  #ifdef CBOR
  payload.reset();
  payload.map(6);
  payload.addNumber(soundValue, "12");
  payload.addNumber(lightValue, "6");
  payload.addNumber(temp, "5");
  payload.addNumber(hum, "11");
  payload.addNumber(pres, "10");
  payload.addInteger(airValue, "13");
  payload.addToQueue(false);
  process();
  #endif

  #ifdef BINARY
  payload.reset();
  payload.addNumber(soundValue);
  payload.addNumber(lightValue);
  payload.addNumber(temp);
  payload.addNumber(hum);
  payload.addNumber(pres);
  payload.addInteger(airValue);
  payload.addToQueue(false);
  process();
  #endif
}

void displaySensorValues()
{
  debugSerial.print("Sound level: ");
  debugSerial.print(soundValue);
  debugSerial.println(" Analog (0-1023)");
      
  debugSerial.print("Light intensity: ");
  debugSerial.print(lightValue);
  debugSerial.println(" Lux");
      
  debugSerial.print("Temperature: ");
  debugSerial.print(temp);
  debugSerial.println(" °C");
      
  debugSerial.print("Humidity: ");
  debugSerial.print(hum);
	debugSerial.println(" %");
      
  debugSerial.print("Pressure: ");
  debugSerial.print(pres);
	debugSerial.println(" hPa");
  
  debugSerial.print("Air quality: ");
  debugSerial.print(airValue);
	debugSerial.println(" Analog (0-1023)");
}
