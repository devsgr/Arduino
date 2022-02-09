// **********************************************************************************
// Dev Shrestha 1/30/2022
// Use do record GPS location once a minute or so to Flash
// Each sentence is about 50 characters long so it should be able to write 524000/50 = 10480 locations or for about a week
// Once memory is full LED will be continusly on
// LED will blink twice for each successful writes
// Last memory location is written in EEPROM so restarting does not overwrite
// 
// that has an onboard SPI Flash chip. This sketch listens to a few serial commands
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <LowPower.h>
TinyGPSPlus gps;
uint16_t expectedDeviceID=0xEF30;
SPIFlash flash(SS_FLASHMEM, 0xEF30);//0xEF30 for windbond 4mbit flash
#define SERIAL_BAUD      9600
#define SERIAL1_BAUD     9600
#define GPSpin           12 

unsigned long Last_Address = 0;
String lat_lon_time;
bool GPSon = false;
///////////////////////

void setup(){

  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(100); //wait until Serial/monitor is opened
  Serial1.begin(SERIAL1_BAUD);
  while (!Serial1) delay(100); //wait until Serial/monitor is opened
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPSpin, OUTPUT); //Make sure to define GPS pin as output or it will not have enough current (due to pull up resistor) to drive the transistor switch
  
  //ensure the radio module CS pin is pulled HIGH or it might interfere!
  pinMode(SS, OUTPUT); digitalWrite(SS, HIGH);


  //Check if falsh is ready
  if (flash.initialize())
  {
    Serial.println("Flash initilaized !");
    //Blink(100, 2);//Blink(int DELAY_MS, byte loops)
  }
  else {
    Serial.print("Flash initialization FAIL, expectedDeviceID(0x");
    Serial.print(expectedDeviceID, HEX);
    Serial.print(") mismatched the read value: 0x");
    Serial.println(flash.readDeviceId(), HEX);
  }

 // Loggin to 4Mb Flash. In order to avoid over writing writing on flash memory, the last location of memory writtten is saved in EEPROM
 // Taking a chance that at start up, EEPROM byte 0 is not 10101010 (=170). This may happen randomly, so to make it fail safe, write EEPROM 0 to value other than 170 and clear falsh before using
 // Most of the time this should work 
  if (EEPROM.read(0) != 170){ //This is the first time chances are Byte 0 is not 170
      EEPROM.write(0,0xaa);  //0xaa = 10101010
      EEPROM.put(1,Last_Address); // EEPROM, starting Byte 1, write Last_Address which is 0 at this time. It will take 4 bytes as Last_address is Unsigend Long Integer
      flash.chipErase(); //Erage the flash
      while(flash.busy()); //Wait until all erased
  }
  else {
      EEPROM.get(1,Last_Address);  //Byte 0 is 170, so falsh has been initialized. Read the Last_address
  }  
    
    
}

void loop(){

if (!GPSon){
  digitalWrite(GPSpin, LOW); //Loginally, looking at transistor switch circuit, GPSpin High should have turned on the GPS, but for some reason it was reversed.
  //digitalWrite(LED_BUILTIN, HIGH); //Mirror the GPS on with LED 15.
  GPSon = true;
  delay(1000); //Give 5 seconds before even trying to read. It may be reduce to 2 seconds as the loop will wait for serial data feed anyways.
}

while (Serial1.available() > 0){
   if (gps.encode(Serial1.read())){  //From TinyGPS encoder
        if (displayInfo()){          // diplayinfo return true if data was successfully written to flash  
          digitalWrite(GPSpin, HIGH); // Turn off GPS 
          digitalWrite(LED_BUILTIN, LOW); //Turn off on board LED
          GPSon = false;   //REset this varaible
          for (int i=0; i <37; i++)  {   //Put the Moteino to low power mode for about 5 minute
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
          }
          
       }
    }
}


}
bool displayInfo()
{
  Serial.print(F("Location: ")); 
  Serial.println(gps.location.isValid()); 
  Serial.print("Address");
  Serial.println( Last_Address);
//hdop < 2 is excellent singal
//age > 1.5 s is too old
  
  if (gps.location.isValid() && gps.time.isValid(), gps.location.isUpdated() && gps.location.age() < 1000 && gps.hdop.hdop() < 1)
  {
    lat_lon_time = String(gps.location.lat(), 6);
    lat_lon_time += ",";
    lat_lon_time += String(gps.location.lng(), 6);
    lat_lon_time += ",";
    lat_lon_time += gps.date.year();
    lat_lon_time += ",";
    lat_lon_time += gps.date.month();
    lat_lon_time += ",";
    lat_lon_time += gps.date.day();
    lat_lon_time += ",";
    lat_lon_time += gps.time.hour();
    lat_lon_time += ",";
    lat_lon_time += gps.time.minute();
    lat_lon_time += ",";
    lat_lon_time += gps.time.second();
    lat_lon_time += ",";
    lat_lon_time += gps.hdop.hdop();     
    lat_lon_time += ",";
    lat_lon_time += gps.satellites.value();
    lat_lon_time += ",";
    lat_lon_time += readVcc();
    lat_lon_time += "\n\n";     //Each sentence is seperated by new line character. Needs two \n as last one is discarded converting to char array
    Serial.println("GPSString: " + lat_lon_time);
   
    Serial.print("GPS length: ");
    Serial.println(lat_lon_time.length());
   
    if (Last_Address < 524000){
      char msg[lat_lon_time.length()]; //Copy all of it to keep one \n. str_len-1 will not copy \n
      lat_lon_time.toCharArray(msg,lat_lon_time.length());
      
      flash.writeBytes(Last_Address, &msg,lat_lon_time.length()-1);
      Last_Address += lat_lon_time.length()-1; 
      EEPROM.put(1, Last_Address);
      //Blink(500, 3); //Blink 3 times after each successful write.
    
    }
    else{
    digitalWrite(LED_BUILTIN,HIGH); //If flash is full, continuously turn on LED
    digitalWrite(GPSpin, HIGH); // Turn off GPS 
    while(true){}   //Do nothing 
    }
    return true;
  }
  else{
    return false;}
  
}
void Blink(int DELAY_MS, byte loops)
{
  while (loops--)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(LED_BUILTIN,LOW);
    delay(DELAY_MS);  
  }
}

long readVcc() 
{ 
long result; 
// Read 1.1V reference against AVcc 
// set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //Arduino UNO or MEGA =2560
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
delay(20); // Wait for Vref to settle 
ADCSRA |= _BV(ADSC); // Start conversion
while (bit_is_set(ADCSRA,ADSC)); // measuring
result = ADCL; // must read ADCL first - it then locks ADCH
result |= ADCH<<8; // unlocks both
Serial.print("Result: ");
Serial.println( result); 
//1125result = 1126400L  / result; // Back-calculate AVcc in mV = 1.1*1024*1000
result = 2877440L  / result; // Back-calculate AVcc in mV = 1.1*1024*1000
return result; // Vcc in millivolts
}
