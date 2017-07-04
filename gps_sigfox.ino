#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

TinyGPS gps;
SoftwareSerial gpsSerial(10, 11); // GPS RX, TX
SoftwareSerial sigfoxSerial(5, 4); // SIGFOX RX, TX

float lat=0;
float lon=0;
unsigned int bumpingLevel=0;
unsigned int bumpingLevel_total = 0;
int bumping_count = 0;
unsigned long lastSent=0;
bool newData = false;
int counter = 0;

void setup() 
{
  delay(2000);
  
  //Serial setup
  Serial.begin(9600);
  //while (!Serial) {}
  sigfoxSerial.begin(9600);
  gpsSerial.begin(9600);

  //Sensor setup
  if (! mma.begin(0x1c)) { // Can't remove these codes, otherwise the data will be wrong.
    Serial.println("Couldnt start"); 
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);

  //wake up SIGFOX
  sigfoxSerial.write("AT"); //wake the module up
  sigfoxSerial.write(10); //send chr(10) as the finish of current command
  Serial.println("Sent AT to wake up");

  lastSent = millis();
}

void loop()
{
  //MARK: GPS data processing
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      if (gps.encode(c)){newData = true;}
    }
  }
  if (newData)
  {
    unsigned long age;
    gps.f_get_position(&lat, &lon, &age);
    Serial.print("LAT=");
    Serial.print(lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat, 6);
    Serial.print(" LON=");
    Serial.print(lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

    
  //MARK: Accelerometer data processing
  sensors_event_t event; 
  mma.getEvent(&event);
  bumpingLevel_total += abs((9.8-event.acceleration.z)*100);
  bumping_count ++;
  

  //MARK: SIGFOX send message
  if(millis()-lastSent>=10000 ){
    bumpingLevel = bumpingLevel_total/bumping_count;
    bumpingLevel_total = 0;
    bumping_count = 0;
    Serial.println(bumpingLevel);
    String msg = toHex(lat)+toHex(lon)+toHex(bumpingLevel);    
    sigfoxSerial.write("AT$P=0");
    sigfoxSerial.write(10);
    delay(100);
    sigfoxSerial.write("AT$GI?");
    sigfoxSerial.write(10);
    delay(100);
    sigfoxSerial.write("AT$RC");
    sigfoxSerial.write(10);
    delay(100);
    sigfoxSerial.write("AT$SF=");
    sigfoxSerial.print(msg); //send
    sigfoxSerial.write(10);
    Serial.println("Sent " + msg);
    lastSent = millis();  
    counter++;
  }

}

String toHex(unsigned int ui) {
  //  Convert the integer to a string of 4 hex digits.
  byte *b = (byte *) &ui;
  String bytes;
  for (int i=0; i<2; i++) {
    if (b[i] <= 0xF) bytes.concat('0');
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String toHex(float f) {
  //  Convert the float to a string of 8 hex digits.
  byte *b = (byte *) &f;
  String bytes;
  for (int i=0; i<4; i++) {
    if (b[i] <= 0xF) bytes.concat('0');
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
