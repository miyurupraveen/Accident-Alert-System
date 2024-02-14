#include <SoftwareSerial.h>
#include <Wire.h>  // Wire library - used for I2C communication
#include <TinyGPS++.h>

#define rxPin 19
#define txPin 18
#define BUZZER 26
#define BUTTON 25
#define RESET_BUTTON 2

SoftwareSerial sim800L(rxPin,txPin); 
SoftwareSerial gpsSerial(16, 17); // RX, TX
TinyGPSPlus gps;

int ADXL345 = 0x53; // The ADXL345 sensor I2C address
int triggerMagnitude = 40;
bool trigflag = true;
bool crash_detected = false;
float X_out, Y_out, Z_out = 0;  // Outputs
unsigned long startTime;
unsigned long crash_time;
unsigned long shock_delay = 12000; //shock delay should be higher than alert delay
unsigned long alert_delay = 10000;

String buff;

//--------------------------------------------------------------------------------------------
//                                        SETUP FUNCTION
//--------------------------------------------------------------------------------------------
void setup()
{
  //--------------------------------------------------------------------------------------------
  //                                        SIM 800L SETUP
  //--------------------------------------------------------------------------------------------
  
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  sim800L.begin(9600);

  gpsSerial.begin(9600);

  Serial.println("Initializing...");
  
  sim800L.println("AT");
  waitForResponse();

  sim800L.println("ATE1");
  waitForResponse();

  sim800L.println("AT+CMGF=1");
  waitForResponse();

  sim800L.println("AT+CNMI=1,2,0,0,0");
  waitForResponse();

  //--------------------------------------------------------------------------------------------
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  digitalWrite(RESET_BUTTON, LOW);
  digitalWrite(BUZZER, LOW);
  delay(20);
  digitalWrite(BUZZER,HIGH);
  delay(200);
  digitalWrite(BUZZER,LOW);
  delay(200);
  digitalWrite(BUZZER,HIGH);
  delay(200);
  digitalWrite(BUZZER,LOW); // Power ON indicator (Two beeps)

  //--------------------------------------------------------------------------------------------
  //                                        ADXL 345 SETUP
  //--------------------------------------------------------------------------------------------

  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);

  //--------------------------------------------------------------------------------------------
  //                                       GPS MODULE SETUP
  //--------------------------------------------------------------------------------------------

  Serial.println("Waiting for Position Fix...");

  // Wait until a valid position fix is obtained
  while (!gps.location.isValid()) {
      while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
      }
  }
  Serial.println("Position Fix found!");
  triggerBuzzer();
}

//--------------------------------------------------------------------------------------------
//                                       LOOP FUNCTION
//--------------------------------------------------------------------------------------------

void loop()
{
  //--------------------------------------------------------------------------------------------
  //                         RESET the ESP32 when pushbutton pressed
  //--------------------------------------------------------------------------------------------
  if (digitalRead(RESET_BUTTON) == LOW) {
      // Reset the ESP32
      ESP.restart();
  }
  //--------------------------------------------------------------------------------------------
  //                   Sending GPS Location when recieved "gps" to SIM 800l
  //--------------------------------------------------------------------------------------------
  while(sim800L.available()){
    buff = sim800L.readString();
    Serial.println(buff);
    buff.trim();
  
    // Check if the received SMS contains "gps"
    if (buff.indexOf("gps") != -1)
    {
      send_GPSLocation();
    }
  }

  //--------------------------------------------------------------------------------------------
  //                               Reading Accelerometer data
  //--------------------------------------------------------------------------------------------

  float oldX = X_out;
  float oldY = Y_out;
  float oldZ = Z_out;
  
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  if (X_out >= 100){
    X_out = X_out - 256;
  }
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256;
  if (Y_out >= 100){
    Y_out = Y_out - 256;
  }
  Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z_out = Z_out/256;
  if (Z_out >= 100){
    Z_out = Z_out - 256;
  }

  float delX = oldX - X_out;
  float delY = oldY - Y_out;
  float delZ = oldZ - Z_out;

  float magnitude = (sqrt(sq(delX) + sq(delY) + sq(delZ)))*20;

  if (magnitude > triggerMagnitude && trigflag){
    digitalWrite(BUZZER, HIGH);
    crash_detected = true;
    crash_time = millis(); 
     
  }

  if (millis() - crash_time >= alert_delay && crash_detected){
    
      digitalWrite(BUZZER, LOW);
      Serial.print("Xa= ");
      Serial.print(X_out);
      Serial.print("   Ya= ");
      Serial.print(Y_out);
      Serial.print("   Za= ");
      Serial.print(Z_out);
      Serial.print("   Magnitude = ");
      Serial.println(magnitude);

      trigflag = false; 
      startTime = millis();

      crash_detected = false;
      crash_time = 0;
      send_sms();
      delay(10000);
      make_call();
         
    
  }

  if(digitalRead(BUTTON) == LOW){
    delay(200);
    digitalWrite(BUZZER, LOW);
    crash_detected = false;
    crash_time = 0;
    trigflag = true;
    startTime = 0;
  }

  if (millis() - startTime >= shock_delay){
    trigflag = true;
    startTime = 0;
  }

  delay(100);
}

////////////////////    sim 800l functions   /////////////////////////

void send_sms(){
  sim800L.print("AT+CMGS=\"+94711845575\"\r");
  waitForResponse();
  
  sim800L.print("Accident alert !  ");
  get_gps();
  sim800L.write(0x1A);
  waitForResponse();
}

void make_call(){
  sim800L.println("ATD+94711845575;");
  waitForResponse();
}

void waitForResponse(){
  delay(1000);
  while(sim800L.available()){
    Serial.println(sim800L.readString());
  }
  sim800L.read();
}

////////////////////    GPS Module functions   /////////////////////////

void get_gps() {
    unsigned long startTime = millis();

    while (millis() - startTime < 5000) {
        while (gpsSerial.available() > 0) {
            char c = gpsSerial.read();
            gps.encode(c);  // Update the TinyGPS object with incoming characters
        }
    }

    if (gps.location.isValid()) {
        // Print Google Maps link
        sim800L.print("Location :  https://www.google.com/maps?q=");
        sim800L.print(gps.location.lat(), 6);
        sim800L.print(",");
        sim800L.println(gps.location.lng(), 6);
    }

    delay(5000);  // Wait for 5 seconds before the next iteration
}

void send_GPSLocation(){
  sim800L.print("AT+CMGS=\"+94711845575\"\r");
  waitForResponse();
  
  get_gps();
  sim800L.write(0x1A);
  waitForResponse();
}

void triggerBuzzer() {
  for (int i=0; i < 4; i++){
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    delay(50);
  }
}
