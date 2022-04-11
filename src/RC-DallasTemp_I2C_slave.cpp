/*
//I2C SLAVE CODE
//I2C Communication between Two Arduino 
*/

#include <Wire.h> //Library for I2C Communication functions
// Include the libraries we need

#include <Arduino.h>        // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

#include <OneWire.h>
#include <DallasTemperature.h>
#include "I2C_Anything.h" // byte strezams any C variable ,Structures, etc

//this xiao is the master
byte sendToSlaveAddress = 8;


/* Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
  void SERCOM1_Handler()
  {
  Serial2.IrqHandler();
  }
*/

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS A2
#define TEMPERATURE_PRECISION 12
#define PIN_LED_13 (13u)
#define PIN_LED PIN_LED_13
#define LED_BUILTIN PIN_LED

#define PIN_LED_RXL (12u)
#define PIN_LED_TXL (11u)
#define PIN_LED2 PIN_LED_RXL
#define PIN_LED3 PIN_LED_TXL

// Sup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

struct
{
  char ver;
  float tubTemp;
  float heaterTemp;
  float addTemp;
} iic_dataStruct;

struct
{
  char ver;
  //expand if needed
} iic_dataStruct_read_master;


bool haveReceivedSlaveData;
uint32_t ClearLEDTime, TempSampleTime_1 = 0, TempSampleTime_2 = 0, TempSampleTime_3 = 0, StatusUpdtTime = 0;
uint32_t StatusUpdtIntvl = 4000, TempSampleInterval = 2000, BootTIme, timeNow;

// arrays to hold device addresses
//DeviceAddress heaterSensorAddress, tubSensorAddress;

// Assign address manually. The addresses below will need to be changed
// to valid device addresses on your bus. Device address can be retrieved
// by using either oneWire.search(deviceAddress) or individually via
// sensors.getAddress(deviceAddress, index)
DeviceAddress tubSensorAddress = {0x28, 0x56, 0x69, 0x49, 0x33, 0x20, 0x01, 0xB6};
DeviceAddress heaterSensorAddress = {0x28, 0xE5, 0x6A, 0x49, 0x33, 0x20, 0x01, 0xA6};

DeviceAddress addSensorAddress = {0x28, 0x12, 0x0D, 0x43, 0x33, 0x20, 0x01, 0x0A};


float tubTemp = 0, heaterTemp = 0, addTemp = 0;

void printAddress(DeviceAddress deviceAddress);

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress);



// called by interrupt service routine when incoming data arrives
void receiveEvent (unsigned int howMany)
 {
 if (howMany >= (sizeof (iic_dataStruct_read_master)))
   {
   I2C_readAnything (iic_dataStruct_read_master);
   haveReceivedSlaveData = true;
   }  // end if have enough data
 }  // end of receiveEvent

// this function is registered as an event, see setup()




// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}
void setup()
{ // Put your setup code here, to run once:
  BootTIme = millis();

  /*  Serial2.begin(115200);
    // Assign pins 10 & 11 SERCOM functionality
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);

  */

  sensors.begin();
  Serial.begin(115200); //initialize serial communication at 115200 bits per second
  Serial.println("Dallas Temperature IC Control Library Demo");
  Wire.begin(); // start I2C mater
                // Start up the library
                // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode())
    Serial.println("ON");
  else
    Serial.println("OFF");

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addressepos and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index

  // report parasite power requirements
  if (!sensors.getAddress(heaterSensorAddress, 0))
    Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(tubSensorAddress, 1))
    Serial.println("Unable to find address for Device 1");
  if (!sensors.getAddress(addSensorAddress, 2))
    Serial.println("Unable to cF address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial1.println("Unable to find address for insideThermometer");
  // assigns the secoesses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(heaterSensorAddress);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(tubSensorAddress);
  Serial.println();

  Serial.print("Device 2 Address: ");
  printAddress(addSensorAddress);
  Serial.println();

  // set the resolution  for  device
  sensors.setResolution(heaterSensorAddress, TEMPERATURE_PRECISION);
  sensors.setResolution(tubSensorAddress, TEMPERATURE_PRECISION);
  sensors.setResolution(addSensorAddress, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(heaterSensorAddress), DEC);
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(tubSensorAddress), DEC);
  Serial.println();

  Serial.print("Device 2 Resolution: ");
  Serial.print(sensors.getResolution(tubSensorAddress), DEC);
  Serial.println();
  TempSampleTime_1 = millis() + 100;
  TempSampleTime_2 = millis() + 100 + TempSampleInterval * 2 / 3;
  TempSampleTime_3 = millis() + 100 + TempSampleInterval / 3;
}
// funct to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{

  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

float getTempF(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);

  return DallasTemperature::toFahrenheit(tempC);
}

void loop()
{
  timeNow = millis();
  /* overflow preventer */

  /*FIRSGT SENSOR SAMPLE  */
  if (TempSampleTime_1 < timeNow)
  {
    TempSampleTime_1 = timeNow + TempSampleInterval;
    Serial.printf("%d, %d",millis(), timeNow) ;
  
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(tubSensorAddress);
    Serial.println("DONE");

    // print the device information
    printData(tubSensorAddress);
    addTemp = getTempF(tubSensorAddress);
  }

  /*SECOND SENSOR SAMPLE  */
  if (TempSampleTime_2 < timeNow)
  {
    
    TempSampleTime_2 = timeNow + TempSampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(heaterSensorAddress);
    Serial.println("DONE");

    // print the device information
    printData(heaterSensorAddress);
    tubTemp= getTempF(heaterSensorAddress);

    // We are going to send the variable value to the object called n0:
    // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
  }

  /*THIRD SENSOR SAMPLE  */
  if (TempSampleTime_3 < timeNow)
  {
    TempSampleTime_3 = timeNow + TempSampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(addSensorAddress);
    Serial.println("DONE");
    // print the device information
    printData(tubSensorAddress);
    heaterTemp = getTempF(addSensorAddress);

    // We are going to send the variable value to the object called n0:
    // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
  }

  if (StatusUpdtTime < timeNow)
  {
    StatusUpdtTime = timeNow + StatusUpdtIntvl;
    ClearLEDTime = timeNow + 10;


    // We are going to send the variable value to the object called n0:
    // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
    // use this variable to keep track of how many
    // bytes we're stuffieng in the transmit buffer
    ///////////////////////////////////////// Stuff buffer with struct"t
    iic_dataStruct.ver = '3';
    iic_dataStruct.tubTemp = tubTemp;
    iic_dataStruct.heaterTemp = heaterTemp; //heaterTemp;
    iic_dataStruct.addTemp = addTemp;       //heaterTemp;

    //sendSize = myTransfer.txObj(iic_dataStruct, sendSize);
    Wire.beginTransmission(sendToSlaveAddress);
    I2C_writeAnything(iic_dataStruct);
    Wire.endTransmission();


 //   digitalWrite(LED_BUILTIN, LOW);
   digitalWrite(PIN_LED2, LOW);
  //  digitalWrite(PIN_LED_TXL, LOW);
  }
  if (ClearLEDTime < timeNow)
  {
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(PIN_LED_TXL, HIGH);
 

  }
}