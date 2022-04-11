/*
*/

//I2C SLAVE CODE
//I2C Communication between Two Arduino
//CircuitDigest
//Pramoth.T

#include <Arduino.h>        // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

#include <OneWire.h>
#include <DallasTemperature.h>
#include "I2C_Anything.h"


byte SlaveAddress = 9;

/*
  Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
  void SERCOM1_Handler()
  {
  Serial2.IrqHandler();
  }
*/

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS A2
#define TEMPERATURE_PRECISION 12
#define PIN_LED_13  (13u)
#define PIN_LED     PIN_LED_13
#define LED_BUILTIN PIN_LED
 
#define PIN_LED_RXL          (12u)
#define PIN_LED_TXL          S(11u)
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL

// Sup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


struct STRUCT {
  char z;
  float tubTemp;
  float heaterTemp;
} iic_dataStruct;

uint32_t TempSampleTime_1 = 0, TempSampleTime_2 = 0, TempSampleTime_3 = 0, StatusUpdtTime = 0;
uint32_t StatusUpdtIntvl = 500, TempSampleInterval = 3500, BootTIme, timeNow;

// arrays to hold device addresses
//DeviceAddress heaterSensorAddress, tubSensorAddress;

// Assign address manually. The addresses below will need to be changed
// to valid device addresses on your bus. Device address can be retrieved
// by using either oneWire.search(deviceAddress) or individually via
// sensors.getAddress(deviceAddress, index)
DeviceAddress heaterSensorAddress = {0x28, 0x55, 0x17, 0x45, 0x33, 0x20, 0x01, 0x32};
DeviceAddress tubSensorAddress =  {0x28, 0xE5, 0x6A, 0x49, 0x33, 0x20, 0x01, 0xA6};

DeviceAddress addSensorAddress = {0x28, 0x56, 0x69, 0x49, 0x33, 0x20, 0x01, 0xB6};

DeviceAddress scratch;
 
volatile float tubTemp = 0, heaterTemp = 0,  addTemp = 0;

void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);

}


 
 
`
void setup()
{ // Put your setup code here, to run once:
  BootTIme = millis();

  /*  Serial2.begin(115200);
    // Assign pins 10 & 11 SERCOM functionality
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
`
  */

  // Wire.begin(SlaveAddress);                          //Begins I2C communication with Slave Address as 8 at pin (A4,A5)
  // Wire.onReceive(receiveEvent);           //Function call when Slave receives value from master
  //  Wire.onRequest(requestEvent);           //Function call when Master request value from Slave
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
 
  sensors.begin();
  Serial.begin(115200); //initialize serial communication at 115200 bits per second
  Serial.println("Dallas Temperature IC  18b22");
  Wire.begin();

  //myTransfer.begin(Wire);

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
    Serial.println("Unable to find address for Device 2");
// method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  oneWire.reset_search();
  // assigns the first address found to insideThermometer
 // if (!oneWire.search(addSensorAddress)) Serial1.printl (n("Unable to find address for insideThermometer");
  //assigns the secoesses we found on the bus
 
 for(int i= 0; i < 4; i++ ) {
  sensors.getAddress( scratch, i);

  printAddress(scratch);
  sensors.getTempF(scratch);
  Serial.printf("Index: %d /n  ",i );
 
 }
 /*
  sensors.getAddress(addSensorAddress,2);
printAddress(addSensorAddress);
*/


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
  TempSampleTime_1 = millis() + 100;
  TempSampleTime_2 = millis() + 100 + TempSampleInterval/ 3;  TempSampleTime_1 = millis() + 100;
  TempSampleTime_3 = millis() + 100 + TempSampleInterval *2 / 3;

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

// function to print the temperature for a deviceg 
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

/*FIRST SENSOR SAMPLE  */
  if (TempSampleTime_1 < timeNow)
  {
    StatusUpdtTime = timeNow + StatusUpdtIntvl;

    Serial.print("Requesting temperatures...");

    TempSampleTime_1 = timeNow + TempSampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(tubSensorAddress);
    
   // print the device information
    printData(tubSensorAddress);
    tubTemp = getTempF(tubSensorAddress);
    Serial.printf(" Temp: %f",tubTemp);
    Serial.println("DONE");  
  }


/*THIRD SENSOR SAMPLE  */
  if (TempSampleTime_3 < timeNow)
  {
    StatusUpdtTime = timeNow + StatusUpdtIntvl;
    TempSampleTime_3 = timeNow + TempSampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(addSensorAddress);
    Serial.println("DONE");

    // print the device information
  
    
    addTemp = getTempF(addSensorAddress);
    printData(heaterSensorAddress);
    Serial.printf(" Temp: %f",addTemp);
    Serial.println("DONE");        

    }

  if (TempSampleTime_2 < timeNow)
  {
    StatusUpdtTime = timeNow + StatusUpdtIntvl;
    TempSampleTime_2 = timeNow + TempSampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(heaterSensorAddress);
    

    heaterTemp = getTempF(heaterSensorAddress);
    
    printData(heaterSensorAddress);
    Serial.printf(" Temp: %f",heaterTemp);
    Serial.println("DONE");        
  }

  if (StatusUpdtTime < timeNow)
  {
    StatusUpdtTime = timeNow + TempSampleInterval;
           digitalWrite(LED_BUILTIN, LOW);
            digitalWrite(PIN_LED2, LOW);
            digitalWrite(PIN_LED3, LOW);          
  
  
              // We are going to send the variable value to the object called n0:
              // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
            // use this variable to keep track of how many
            // bytes we're stuffieng in the transmit buffer
            ///////////////////////////////////////// Stuff buffer with struct"t
            iic_dataStruct.z ='3';
            iic_dataStruct.tubTemp = tubTemp;
            iic_dataStruct.heaterTemp = heaterTemp; //heaterTemp;
            //sendSize = myTransfer.txObj(iic_dataStruct, sendSize);
            Wire.beginTransmission(8);
            I2C_writeAnything(iic_dataStruct);
            Wire.endTransmission();
          //delay(200);`m``

            digitalWrite(PIN_LED2, HIGH);
            digitalWrite(LED_BUILTIN, HIGH);
            digitalWrite(PIN_LED3, HIGH);     
  }

}
