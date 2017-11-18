/*****************************************************************************

  Prototype I2C interface to Aibo Battery LCD Version 
  Read Battery Data from Aibo ERS2xx, ERS3xx and ERS7 series.
  Last update: AndreasVan 2017-11-17 Vers.1.7

  Micro controller Arduino Uno R3

  Arduino analog input 4 - I2C SDA
  Arduino analog input 5 - I2C SCL
 
  Arduino Uno Pinout 
  https://forum.arduino.cc/index.php?topic=146315.0
  
  Arduino 1602 LCD Shield
  https://www.itead.cc/wiki/Arduino_1602_LCD_Shield
   
  Read the information via 2-Wire SMBus 
  http://www.ti.com/lit/ds/symlink/bq2060a.pdf
  
  based on PackProbe:
 
  http://powercartel.com/projects/packprobe/

  Depends on SoftI2CMaster http://playground.arduino.cc/Main/SoftwareI2CLibrary

*****************************************************************************/

//  Arduino Data = A4
#define SDA_PORT PORTC
#define SDA_PIN 4

// Arduino CLK = A5
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_SLOWMODE 1
#include <SoftI2CMaster.h>
#include <Wire.h>

// standard I2C address for Smart Battery packs
byte deviceAddress = 11;

#include <LiquidCrystal.h>

// Status LED
int LED1 = A1; //LED pin 1
int LED2 = A0; //LED pin 2

int x = 0;
int currx = 1023;
String btnStr = "None";

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // LCD Shield Andreas
// LiquidCrystal lcd(8, 9, 5, 4, 3, 2); // 16x2 LCD Andreas SD Card
// LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // 16x2 LCD Chris

// Standard and common non-standard Smart Battery commands
#define BATTERY_MODE             0x03
#define TEMPERATURE              0x08
#define VOLTAGE                  0x09
#define CURRENT                  0x0A
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define REMAINING_CAPACITY       0x0F
#define FULL_CHARGE_CAPACITY     0x10
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define DESIGN_CAPACITY          0x18
#define DESIGN_VOLTAGE           0x19
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define SERIAL_NUM               0x1C
#define MFG_NAME                 0x20
#define DEV_NAME                 0x21
#define CELL_CHEM                0x22
#define MFG_DATA                 0x23
#define CELL1_VOLTAGE            0x3F
#define CELL2_VOLTAGE            0x3E
#define CELL3_VOLTAGE            0x3D
#define CELL4_VOLTAGE            0x3C
#define STATE_OF_HEALTH          0x4F

#define bufferLen 32
uint8_t i2cBuffer[bufferLen];

void setup()
{
  pinMode(LED1, OUTPUT); //set the LED pin as OUTPUT
  pinMode(LED2, OUTPUT); //set the LED pin as OUTPUT
//  Wire.begin();
  Serial.begin(115200);  // start serial for output
  Serial.println(i2c_init());
  pinMode(22,INPUT_PULLUP);  //22
  pinMode(23,INPUT_PULLUP);  //23
// set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
// Print a message to the LCD.
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Aibo Batterytool");
  lcd.setCursor(0,1);
  lcd.print("by Yaba2017 V1.7");
  delay(3003);
  digitalWrite(LED1, HIGH);
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(" I2C Inialized ");
  delay(2000                  );
  digitalWrite(LED1, LOW);
  Serial.println("Get Started");
  
  i2c_init();
  Serial.println("I2C Inialized");
  scan();
}

int fetchWord(byte func)
{
  i2c_start(deviceAddress<<1 | I2C_WRITE);
  i2c_write(func);
  i2c_rep_start(deviceAddress<<1 | I2C_READ);
  byte b1 = i2c_read(false);
  byte b2 = i2c_read(true);
  i2c_stop();
  return (int)b1|((( int)b2)<<8);
}

uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen ) 
{
  uint8_t x, num_bytes;
  i2c_start((deviceAddress<<1) + I2C_WRITE);
  i2c_write(command);
  i2c_rep_start((deviceAddress<<1) + I2C_READ);
  num_bytes = i2c_read(false); // num of bytes; 1 byte will be index 0
  num_bytes = constrain(num_bytes,0,blockBufferLen-2); // room for null at the end
  for (x=0; x<num_bytes-1; x++) { // -1 because x=num_bytes-1 if x<y; last byte needs to be "nack"'d, x<y-1
    blockBuffer[x] = i2c_read(false);
  }
  blockBuffer[x++] = i2c_read(true); // this will nack the last byte and store it in x's num_bytes-1 address.
  blockBuffer[x] = 0; // and null it at last_byte+1
  i2c_stop();
  return num_bytes;
}

void scan()
{
  byte i = 0;
  for ( i= 0; i < 127; i++  )
  {
    Serial.print("Address: 0x");
    Serial.print(i,HEX);
    bool ack = i2c_start(i<<1 | I2C_WRITE); 
    if ( ack ) {
      Serial.println(": OK");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Address: 0x");
      lcd.setCursor(11,0);
      lcd.print(i,HEX);
      lcd.setCursor(14,0);
      lcd.print("OK");
      lcd.setCursor(0,1);
      lcd.print(" Battery found!");
      delay(2002);
      Serial.flush();
    }  
    else {
      Serial.println(": -");
 //     lcd.clear();   
 //     lcd.setCursor(0,0);
 //     lcd.print("No Battery");   
 //     delay(2002);
      Serial.flush();      
    }
    i2c_stop();
  }
}

void loop()
{
  uint8_t length_read = 0;
  currx = x;
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Reading Data");
  lcd.setCursor(0,1);
  digitalWrite(LED2, HIGH);
  lcd.print("..");
  delay(150);
  digitalWrite(LED2, LOW);
  lcd.print("....");
  delay(150);
  digitalWrite(LED2, HIGH);
  lcd.print("......");
  delay(150);
  digitalWrite(LED2, LOW);
  lcd.print(".........");
  delay(150);
  digitalWrite(LED2, HIGH);
  lcd.print("...........");
  delay(200);
  digitalWrite(LED2, LOW);

  Serial.print("Manufacturer Name: ");
  length_read = i2c_smbus_read_block(MFG_NAME, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println("");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Manufacturer:");
  lcd.setCursor(0,1);
  lcd.write(i2cBuffer, length_read);
  delay(2002);

  Serial.print("Device Name: ");
  length_read = i2c_smbus_read_block(DEV_NAME, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println("");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Device Name:");
  lcd.setCursor(0,1);
  lcd.write(i2cBuffer, length_read);
  delay(2002);

  Serial.print("Chemistry ");
  length_read = i2c_smbus_read_block(CELL_CHEM, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println("");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Chemistry :");
  lcd.setCursor(0,1);
  lcd.write(i2cBuffer, length_read);
  delay(2002);
 
//  String formatted_date = "Manufacture Date (Y-M-D): ";
  String formatted_date = "Date: ";
  int mdate = fetchWord(MFG_DATE);
  int mday = B00011111 & mdate;
  int mmonth = mdate>>5 & B00001111;
  int myear = 1980 + (mdate>>9 & B01111111);
  formatted_date += myear;
  formatted_date += "-";
  formatted_date += mmonth;
  formatted_date += "-";
  formatted_date += mday;
  Serial.println(formatted_date);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Manufacture");
  lcd.setCursor(0,1);
  lcd.print(formatted_date);
  delay(2002);
  
  Serial.print("Manufacturer Data ");
  length_read = i2c_smbus_read_block(MFG_DATA, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  Serial.println("");

  Serial.print("Design Capacity: " );
  Serial.println(fetchWord(DESIGN_CAPACITY));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Design Capacity:");
  lcd.setCursor(0,1);
  lcd.print(fetchWord(DESIGN_CAPACITY));
  lcd.setCursor(5,1);
  lcd.print("mAh");
  delay(2002);
  
  Serial.print("Full Charge Capacity: " );
  Serial.println(fetchWord(FULL_CHARGE_CAPACITY));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Full Capacity:");
  lcd.setCursor(0,1);
  lcd.print(fetchWord(FULL_CHARGE_CAPACITY));
  lcd.setCursor(5,1);
  lcd.print("mAh");
  delay(2002);
  
  Serial.print("Remain Capacity: " );
  Serial.println(fetchWord(REMAINING_CAPACITY));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Remain Capacity:");
  lcd.setCursor(0,1);
  lcd.print(fetchWord(REMAINING_CAPACITY));
  lcd.setCursor(5,1);
  lcd.print("mAh");
  delay(2002);

  Serial.print("Cycle: " );
  Serial.println(fetchWord(CYCLE_COUNT));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Charge Cycle");
  lcd.setCursor(0,1);
  lcd.print("Count:");
  lcd.setCursor(7,1);
  lcd.print(fetchWord(CYCLE_COUNT));
  delay(2002);
      
  Serial.print("Design Voltage: " );
  Serial.println(fetchWord(DESIGN_VOLTAGE));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Design Voltage:");
  lcd.setCursor(0,1);
  lcd.print((float)fetchWord(DESIGN_VOLTAGE)/1000);
  lcd.setCursor(5,1);
  lcd.print("Volt");
  delay(2002);
  
  Serial.print("Serial Number: ");
  Serial.println(fetchWord(SERIAL_NUM));

  Serial.print("Specification Info: ");
  Serial.println(fetchWord(SPEC_INFO));
 
  Serial.print("Voltage: ");
  Serial.println((float)fetchWord(VOLTAGE)/1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Voltage:");
  lcd.setCursor(0,1);
  lcd.print((float)fetchWord(VOLTAGE)/1000);
  lcd.setCursor(5,1);
  lcd.print("Volt");
  delay(2002);
  
  Serial.print("Relative Charge(%): ");
  Serial.println(fetchWord(RELATIVE_SOC));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Relative Charge:");
  lcd.setCursor(0,1);
  lcd.print(fetchWord(RELATIVE_SOC));
  lcd.setCursor(3,1);
  lcd.print("%");
  delay(2002);
  
  Serial.print("Absolute Charge(%): ");
  Serial.println(fetchWord(ABSOLUTE_SOC));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Absolute Charge:");
  lcd.setCursor(0,1);
  lcd.print(fetchWord(ABSOLUTE_SOC));
  lcd.setCursor(3,1);
  lcd.print("%");
  delay(2002);
    
  Serial.print("Minutes remaining for full charge: ");
  Serial.println(fetchWord(TIME_TO_FULL));

  Serial.print("Cell 1 Voltage: ");
  Serial.println(fetchWord(CELL1_VOLTAGE));
  Serial.print("Cell 2 Voltage: ");
  Serial.println(fetchWord(CELL2_VOLTAGE));
  Serial.print("Cell 3 Voltage: ");
  Serial.println(fetchWord(CELL3_VOLTAGE));
  Serial.print("Cell 4 Voltage: ");
  Serial.println(fetchWord(CELL4_VOLTAGE));
  
  Serial.print("State of Health: ");
  Serial.println(fetchWord(STATE_OF_HEALTH));
  
  Serial.print("Charging Current: ");
  Serial.println(fetchWord(CHARGING_CURRENT));
  
  Serial.print("Charging Voltage: ");
  Serial.println(fetchWord(CHARGING_VOLTAGE));

  Serial.print("Temp: ");
  unsigned int tempk = fetchWord(TEMPERATURE);
  Serial.println((float)tempk/10.0-273.15);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temperature:");
  lcd.setCursor(0,1);
  lcd.print((float)tempk/10.0-273.15);
  lcd.setCursor(6,1);
  lcd.print("Celsius");
  delay(2002);

  Serial.print("Current (mA): " );
  Serial.println(fetchWord(CURRENT));
  
  Serial.println(".");
  delay(1000);
}
