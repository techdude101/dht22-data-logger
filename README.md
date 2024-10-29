# dht22-data-logger
Arduino Nano + DHT22 Data Logger

Logs temperature to Arduino Nano EEPROM every minute  
Data is retrieved via USB Serial port

Instructions:  
  Hardware
  1. Solder a link wire between USB D+ and Arduino header pin A1
  
  Programming
  1. Program the Arduino
  2. Open serial monitor / putty etc... with BAUD rate set to 9600
  3. Type E then enter to erase EEPROM

  Data retrieval
  1. Connect Arduino Nano to a PC/laptop via USB cable
  2. Open serial monitor / putty etc... with BAUD rate set to 9600
  3. Type D then enter to display the logged data
