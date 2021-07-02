/*
   Project:     DHT22 Data Logger

   Description: Logs temperature to EEPROM
   Version:     0.1.0
   Hardware:    Arduino Nano
                Wire connection between USB D+ and A1 added
                DHT22 module -> Arduino Nano
                1 (VCC)      -> 5V
                2 (Data)     -> D7
                3 (GND)      -> GND
*/

/*
  Scenario 1: First power on - no data in EEPROM
   1. Erase EEPROM
   2. Write current EEPROM address to EEPROM
   3. Start logging

  Scenario 2: User unplugs device from computer, powers the device by an external power source
   1. Read the current EEPROM address from EEPROM
   2. Start logging

  Scenario 3: Data logger has been logging, user removes external power source then plugs the device into a computer to retrieve the data
   1. Show the serial menu
   2. If user enters the menu option to download
     - Read the data from EEPROM
     - Print the data to the serial console

   Scenario 4: Memory full
    1. Stop logging
    2. Flash onboard LED

  Scenario 5: Defective sensor / faulty wiring
    1. Stop logging
    2. Flash onboard LED

  Scenario 6: Low battery
    1. Flash onboard LED every 5 seconds

  Scenario 7: Data corrupted
    1. Stop logging
    2. Flash onboard LED

  Scenario 8: Power failure while writing EEPROM
*/

// Libraries
#include <DHT.h>
#include <EEPROM.h>
#include <LowPower.h>

// Constants
#define DHTPIN 7
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino

// Global variables
const int ONBOARD_LED = 13;
bool USB_DETECTED = false;
const int EEPROM_ADDRESS = 0;
const int EEPROM_DATA_START_ADDRESS = 1;
float temp; //Stores temperature value
int eeprom_current_address = EEPROM_DATA_START_ADDRESS;
bool memory_full = false;

// Interrupt service routines
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  int usbDataPlus = digitalRead(A1);

  if (usbDataPlus == LOW) {
    USB_DETECTED = true;
  }
}

// Functions
void(* resetFunc) (void) = 0; //declare reset function at address 0

int readCurrentAddressFromEEPROM() {
  int address = 0;
  EEPROM.get(EEPROM_ADDRESS, address);
  return address;
}

bool writeCurrentAddressToEEPROM(int address) {
  if (address >= EEPROM.length()) return false;
  EEPROM.put(EEPROM_ADDRESS, address);
  int writtenAddress = 0;
  EEPROM.get(EEPROM_ADDRESS, writtenAddress);
  return (writtenAddress == address);
}

int writeTemperatureToEEPROM(float temperature, int address) {
  if (address >= EEPROM.length()) return 0;
  EEPROM.put(address, temperature);
  return sizeof(float);
}

float readTemperatureFromEEPROM(int address) {
  if (address >= EEPROM.length()) return 0.0f;
  float temp = 0.0f;
  return EEPROM.get(address, temp);
}

void eraseEEPROM() {
  for (unsigned int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void printTemperature(float temperature, int index) {
  Serial.print(index);
  Serial.print(", ");
  Serial.print(temperature);
  Serial.println(" C");
}

void printAllTemperatures() {
  if (eeprom_current_address <= 0) Serial.println("No data!");
  for (int i = EEPROM_DATA_START_ADDRESS, index = 1; i < eeprom_current_address; i += sizeof(float), index += 1) {
    float temp = readTemperatureFromEEPROM(i);
    printTemperature(temp, index);
  }
}

void serialMenu() {
  Serial.println("A - display current EEPROM address");
  Serial.println("D - download / display data");
  Serial.println("E - erase data");
  Serial.println("L - exit and start logging");

  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'd':
      case 'D':
        Serial.println("Download");
        printAllTemperatures();
        break;
      case 'e':
      case 'E':
        Serial.println("Erasing EEPROM");
        eraseEEPROM();
        eeprom_current_address = EEPROM_DATA_START_ADDRESS;
        writeCurrentAddressToEEPROM(eeprom_current_address);
        resetFunc();
        Serial.println("Erase complete!");
        break;
      case 'l':
      case 'L':
        Serial.println("Starting logging");
        break;
      case 'a':
      case 'A':
        Serial.print("Current EEPROM address: ");
        Serial.println(readCurrentAddressFromEEPROM());
        break;
    }
  }
}
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void pciRemove(byte pin)
{
  *digitalPinToPCMSK(pin) &= bit (~(digitalPinToPCMSKbit(pin)));  // enable pin
  PCIFR  &= bit (~(digitalPinToPCICRbit(pin))); // clear any outstanding interrupt
  PCICR  &= bit (~(digitalPinToPCICRbit(pin))); // enable interrupt for the group
}

void ledOn() {
  digitalWrite(ONBOARD_LED, HIGH);
}

void ledOff() {
  digitalWrite(ONBOARD_LED, LOW);
}

void sleepTest() {
  while (1) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void saveTemperature(float data) {
  if ((eeprom_current_address + sizeof(float)) < EEPROM.length() && (memory_full == false)) {
    eeprom_current_address += writeTemperatureToEEPROM(temp, eeprom_current_address);
    if (!writeCurrentAddressToEEPROM(eeprom_current_address)) {
      Serial.println("Error writing current EEPROM address");
    }
  } else {
    memory_full = true;
  }
}

void flashLed(int pin, unsigned int on_time_ms) {
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);
  delay(on_time_ms);
  digitalWrite(pin, LOW);
}

void setup() {
  //  sleepTest();
  pinMode(ONBOARD_LED, OUTPUT);
  Serial.begin(9600);

  dht.begin();
  Serial.println("Startup...");

  eeprom_current_address = readCurrentAddressFromEEPROM();

  Serial.print("EEPROM current address: ");
  Serial.println(eeprom_current_address);
  pciSetup(A1); // Enable interrupt on pin
  delay(2000);
  pciRemove(A1); // Disable interrupt to prevent wakeup due to noise etc...
}

void loop() {
    const byte sample_interval_in_seconds = 60;
  unsigned int second_count = 0;

  if (memory_full) {
    flashLed(ONBOARD_LED, 500);
  } else {
    // Read temperature
    temp = dht.readTemperature();

    // Save temperature
    saveTemperature(temp);
  }

  // Wait until time to read next sample
  // 1. USB powered via computer - display serial menu if user presses a key
  // 2. USB powered via power bank / battery - enter low power mode
  while (second_count < sample_interval_in_seconds) {
    if (USB_DETECTED) {
      if (Serial.available() > 1) {
        serialMenu();
        delay(1);
      }
      delay(1000);
    } else {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    }
    second_count++;
  }
}
