/*
Smart AC Meter V1.0
Author - Soumyabrata Debnath (Electromaniac) 
*/
#define BLYNK_TEMPLATE_ID "************************"
#define BLYNK_TEMPLATE_NAME "**********************"
#define BLYNK_DEVICE_NAME "************************"
#define BLYNK_AUTH_TOKEN "*************************"

//Libraries Initialization
//Wifi Specific 
#include <WiFiS3.h> 
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <BlynkSimpleWifi.h>   
//Pheripherials Specific
#include <SD.h>
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <ZMPT101B.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>

//Pin Mappings

#define NC          0 //Rx
#define NC          1 //Tx
#define vbSense     2
#define relayPin    3 
#define NC          4
#define NC          5
#define NC          6
#define fSense      7
#define NC          8
#define tSense      9
#define chipSelect 10
#define MOSI       11		
#define MISO       12
#define SCK        13
#define vSense     14
#define iSense     15
#define mSense     16
#define aLED       17
#define NC         18 //SDA
#define NC         19 //SCL

char auth[] = BLYNK_AUTH_TOKEN; 
char ssid[] = "Your WiFi SSID";       // WiFi SSID
char pass[] = "Your WiFi password";   // WiFi password

//Peripherals Initialization 
RTC_DS3231 rtc;
WiFiUDP ntpUDP;
BlynkTimer timer;
DHT dht(tSense, DHT11);
ZMPT101B voltageSensor(vSense, 50.0);
LiquidCrystal_I2C lcd(0x27,16,2);  // lcd(address,column,row)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, aLED, NEO_GRB + NEO_KHZ800);
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // India Timezone (GMT+5:30)

// Variables Initialization
         bool   relayFlag = false;
	float  voltageRMS =   0.0;
	float  currentRMS =   0.0;
	float        watt =   0.0;
	float   eConsumed =   0.0; // Total energy consumed in kWh
          int   unitCount =     0; // To track 1 kWh units
         bool  data_saved = false; // Flag to ensure data is saved once at 9 AM
          int      mState =  HIGH; // Variable to store Hall sensor state        
          int     vbState =  HIGH; // Variable to store Vibration sensor state
	  int      fState =  HIGH; // Variable to store Flame sensor state 
          int  mStateFlag =     0; // Variable to store Hall sensor state        
	  int vbStateFlag =     0; // Variable to store Vibration sensor state
          int  fStateFlag =     0; // Variable to store Flame sensor state    		  
          int    ledState =   LOW; // LED on/off state
    const int    unitAddr =     0; // EEPROM address to store unit count
unsigned long    lastTime =     0;
unsigned long CprevMillis =     0; // Store the last time the function ran

void send_vSense() {
    voltageRMS = voltageSensor.getRmsVoltage(100);
    Blynk.virtualWrite(V0, voltageRMS);                    // Send voltage to Blynk (Virtual Pin V0)
}
void send_iSense() {
	float isumOfSquares = 0.0;    
    for(int i = 0; i < 1000; i++) {
       int isensorValue =  analogRead(iSense);             // Read sensor value
         float iVoltage = (isensorValue * 5000.0) / 4096.0;// Convert ADC value to voltage      
          float current = (iVoltage - 2500) / 45.0;        // Convert voltage to current             
         isumOfSquares +=  current * current;              // Square each current value and sum them up		  	
    }      
    float imeanOfSquares = isumOfSquares / 1000;           // Calculate the mean of the squares      
    currentRMS = sqrt(imeanOfSquares);                     // Calculate RMS current 	
    if (currentRMS <= 0.1) {
        currentRMS = 0;
    }	
	Blynk.virtualWrite(V1, currentRMS);                // Send current to Blynk (Virtual Pin V1)    
}
void send_watt() { 
    watt = voltageRMS * currentRMS;      
    Blynk.virtualWrite(V2, watt);                          // Send watt to Blynk (Virtual Pin V2)
}
void send_unit() {    
    Serial.println(unitCount);                             
    Blynk.virtualWrite(V3, unitCount);                     // Send unit to Blynk (Virtual Pin V3)
}
void send_tSense() {
    float t = dht.readTemperature() - 10;                  // Read temperature in Celsius
    float h = dht.readHumidity();                          // Read the humidity
    Serial.println(t);                                     // Print temperature to serial monitor
    Serial.println(h);                                     // Print humidity to serial monitor
    Blynk.virtualWrite(V4, t);                             // Send temperature to Blynk (Virtual Pin V4)
    Blynk.virtualWrite(V5, h);                             // Send humidity to Blynk (Virtual Pin V5)
}
void send_mSense() {
    mState = digitalRead(mSense);                          // Read hall sensor state
    if (mState == LOW) {                                   // If a magnet is detected (hall sensor output LOW)
        digitalWrite(relayPin, LOW);                       // Turn relay OFF
        Serial.println("Magnet detected: Relay OFF"); 
        mStateFlag = 1;
	    relayFlag = false;     
        Blynk.virtualWrite(V9, relayFlag);
	}    
	Serial.print("mState:");	
    Serial.println(mState);                                // Print hallState to serial monitor   
    Blynk.virtualWrite(V6, mStateFlag);                    // Send hallState to Blynk 
}
void send_vbSense() {    
    vbState = digitalRead(vbSense);                        // Read vibration sensor state
	if (vbState == LOW) {                              // If a vibration is detected (vibration sensor output LOW)
        digitalWrite(relayPin, LOW);                       // Turn relay OFF
        Serial.println("Vibration detected: Relay OFF");
        vbStateFlag = 1;
		relayFlag = false;     
        Blynk.virtualWrite(V9, relayFlag);       
	} 	
	Serial.print("vbState:");
	Serial.println(vbState);                           // Print vbState to serial monitor   
    Blynk.virtualWrite(V7, vbStateFlag);                   // Send vbState to Blynk 
}
void send_fSense() {
	fState = digitalRead(fSense);    
    if (fState == LOW) {                                   // Assuming LOW indicates flame detection
        digitalWrite(relayPin, LOW);                       // Turn off relay
        Serial.println("Flame detected! Relay turned off.");
		fStateFlag = 1;
		relayFlag = false;     
        Blynk.virtualWrite(V9, relayFlag);
    }     
	Serial.print("Flame State:");
	Serial.println(fState);                            // Print fState to serial monitor   
    Blynk.virtualWrite(V8, fStateFlag);                    // Send fState to Blynk 
}
BLYNK_WRITE(V9) {
    relayFlag = param.asInt();                             // Update flag based on switch value from the app
    Serial.print("Relay state changed from Blynk: ");
    Serial.println(relayFlag);
    if (relayFlag) {
        digitalWrite(relayPin, HIGH);                      // Turn relay ON
		mStateFlag = 0;
	   vbStateFlag = 0;
		fStateFlag = 0;
    } 
    else {
        digitalWrite(relayPin, LOW);                       // Turn relay ON
    }
}
BLYNK_WRITE(V10) {
    int buttonState = param.asInt();
    if (buttonState == 1) {                                // Button pressed
        unitCount = 0;                                     // Reset unit count
        writeUnitCount(unitCount);                         // Update EEPROM
        Blynk.virtualWrite(V3, unitCount);                 // Update unit count on Blynk display
        Serial.println("Unit count reset to 0");
    }
}
void time_update() {
    timeClient.update();                                   // Update NTP time  
    unsigned long epochTime = timeClient.getEpochTime();   // Get current time from NTP  
    DateTime currentTime = DateTime(epochTime);            // Convert epoch time to DateTime object 
    rtc.adjust(currentTime);                               // Set time on DS3231
    Serial.println("Time updated from NTP.");
}
void saveDataToCSV() {
    File dataFile = SD.open("units.csv", FILE_WRITE);      // Open the file
    if (dataFile) {    
        DateTime now = rtc.now();
        dataFile.print(now.year(), DEC);
        dataFile.print('/');
        dataFile.print(now.month(), DEC);
        dataFile.print('/');
        dataFile.print(now.day(), DEC);
        dataFile.print(",");    
        dataFile.println(unitCount); 
        dataFile.close();  // Close the file
        Serial.println("Data saved to SD card as CSV.");
    } 
    else {
    Serial.println("Error opening file.");
    }
}
void setLEDColor(int red, int green, int blue) {
    strip.setPixelColor(0, strip.Color(red, green, blue)); // Set color of first LED
    strip.show();                                          // Update LED strip
}
void writeUnitCount(int count) {
    byte lowByte  = count & 0xFF;                          // Lower 8 bits
    byte highByte = (count >> 8) & 0xFF;                   // Upper 8 bits
    EEPROM.write(unitAddr, lowByte);                       // Write lower byte to EEPROM
    EEPROM.write(unitAddr + 1, highByte);                  // Write upper byte to next address
}
int readUnitCount() {
    byte lowByte  = EEPROM.read(unitAddr);                 // Read lower byte from EEPROM
    byte highByte = EEPROM.read(unitAddr + 1);             // Read upper byte from EEPROM
    return (highByte << 8) | lowByte;                      // Combine the two bytes
}	
void setup() {
    Serial.begin(9600);	 	
    pinMode(iSense, INPUT);
    pinMode(fSense, INPUT);
    pinMode(vbSense,INPUT);
    pinMode(mSense, INPUT_PULLUP);
    pinMode(relayPin, OUTPUT);                             // Set relay pin as output    
    lcd.init();           
    lcd.home();
    lcd.backlight();
    Wire.begin();
    rtc.begin();	
    analogReadResolution(12);
    voltageSensor.setSensitivity(500.0);    
    Blynk.begin(auth, ssid, pass);                         // Connect to Blynk
    dht.begin();                                           // Initialize DHT sensor      
    Blynk.syncVirtual(V9);
    timer.setInterval(1000L, send_vSense);                 // Send voltage data every 1 seconds
    timer.setInterval(1000L, send_iSense);                 // Send current data every 1 seconds
    timer.setInterval(1000L, send_watt);                   // Send Watt    data every 1 seconds
    timer.setInterval(1000L, send_unit);                   // Send Unit counted every 1 seconds
    timer.setInterval(1000L, send_tSense);                 // Send T/H     data every 1 seconds
    timer.setInterval(1000L, send_mSense);                 // Send Magnet  data every 1 seconds
    timer.setInterval(1000L, send_vbSense);                // Send Vibration    every 1 seconds
    timer.setInterval(1000L, send_fSense);                 // Send Flame data   every 1 seconds	
    unitCount = readUnitCount();                           // Retrieve last saved unit count from EEPROM
    Serial.print("Restored Unit Count: ");
    Serial.println(unitCount);         
    timeClient.begin();
    time_update();    
    File dataFile = SD.open("units.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Date,Units Consumed");           // Add a header row
        dataFile.close();
    }
    strip.begin();                                         // Initialize the strip
    strip.show();                                          // Initialize all pixels to 'off'  
    setLEDColor(0, 0, 10); 
    delay(2000);                                           // Keep it on for 2 seconds    
    setLEDColor(0, 0,  0);
}

void loop() {
    Blynk.run();  // Run Blynk
    timer.run();  // Run timer    
    int state = digitalRead(relayPin);	
    if (state == LOW){	
	setLEDColor(20, 0, 0);
    }
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 1000) {
             lastTime  = currentTime;            
        float   power  = voltageRMS * currentRMS;          // Instantaneous power in watts   
        float   energy = (power / 1000.0) / 3600.0;        // Convert power to kWh over 1 second             
            eConsumed += energy;                           // Accumulate energy consumed    
        if (eConsumed >= 1.0)                              // Check if 1 kWh (1 unit) has been consumed
	{
            unitCount +=   1;                              // Increment 1 unit
            eConsumed -= 1.0;                              // Reset energy consumption by 1 kWh
            writeUnitCount(unitCount);                     // Store updated count in EEPROM
	    Serial.print("1 Unit Consumed. Total Units: ");
            Serial.println(unitCount);
        }    
        Serial.print("Voltage: ");
        Serial.print(voltageRMS);
        Serial.print("V, Current: ");
        Serial.print(currentRMS);
        Serial.print("A, Power: ");
        Serial.print(power);
        Serial.print("W, Energy: ");
        Serial.print(eConsumed);
        Serial.println(" kWh");
      
	lcd.init();
        lcd.clear();
	lcd.setCursor(0, 0);           
        lcd.print(voltageRMS,0);
        lcd.print("V");	
        lcd.setCursor(5, 0);           
        lcd.print(currentRMS,2);
        lcd.print("A");
        lcd.setCursor(11, 0);           
        lcd.print(power,0);
        lcd.print("W");		
        lcd.setCursor(0, 1);
        if (state == LOW){	
	    lcd.print("Cuttoff");
	}
        else{
            lcd.print(eConsumed,3);
	    lcd.print("kWh");
        }	
	int unitPosition = 15 - String(unitCount).length(); 
        lcd.setCursor(unitPosition, 1);
        lcd.print(unitCount);
        lcd.print("U");        
    }	
    unsigned long currentMillis = millis();
    static unsigned long previousMillis = 0;
    static unsigned long ledOnMillis = 0;
    static bool ledState = LOW;
    if (watt > 0){        
        int flashDelay = map(watt, 0, 5000, 2000, 100);          
        if (currentMillis - previousMillis >= flashDelay) {
            previousMillis = currentMillis;    
            if (ledState == LOW) {
                setLEDColor(0, 20, 0);                     // Set LED to red
                ledState = HIGH;
                ledOnMillis = currentMillis;               // Record the time LED turned on
            } 
        }
        if (ledState == HIGH && (currentMillis - ledOnMillis >= 100)) {
            setLEDColor(0, 0, 0);                          // Turn off LED
            ledState = LOW;
        }
    }
    if (watt <= 0 && state == HIGH){
        setLEDColor(0, 0, 0);
    }
    if (currentMillis - CprevMillis >= 60000) {
        CprevMillis = currentMillis;                       // Save the last time the function was called
        time_update();                                     // Run the time update function
    }
    DateTime now = rtc.now();                              // Get current time from RTC	
    if (now.hour() == 9 && now.minute() == 0 && now.second() == 0 && !data_saved) {
        saveDataToCSV();                                   // Save data to SD card
        data_saved = true;                                 // Ensure data is saved only once at 9am
    }  
    if (now.hour() != 9) {
        data_saved = false;
    }	   
}
