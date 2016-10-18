/* Hologram Dash GPS and Temperature Sensor Sketch
  * Modified by Hologram for the Hologram Dash
  * This is based off of the Adafruit Arduino Due example at:
  * https://github.com/adafruit/Adafruit_GPS/blob/master/examples/due_parsing/due_parsing.ino
  * This modified Hologram version sends the latitude and longitude data back to the
  * Hologram cloud
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  * 
  */

#include <Adafruit_GPS.h>
#include <DHT.h>

#define Serial SerialUSB
#define mySerial Serial0
#define DHTPIN R04     // DHT Sensor 
#define DHTTYPE DHT11   // DHT 11
#define RLED D28 // Red LED
#define GLED D30 // Green LED

//Change this to change how often the GPS data is printed and sent to the cloud
#define GPS_SERIAL_SEND_TIME 30000
#define GPS_CLOUD_SEND_TIME 3600000
#define MIN_FIRST_SEND_TIME 120000

#define LED_DATA_INDICATOR_DURATION 200
#define LED_DATA_INDICATOR_FLICKER_OFF_DURATION 50
#define LED_DATA_INDICATOR_FLICKER_ON_DURATION 5

#define DASH_CLOUD_SLEEP_INTERVAL 50 // in minutes
#define DASH_BATTERY_THRESHOLD 10 // in percentage

Adafruit_GPS GPS(&mySerial);

DHT dht(DHTPIN, DHTTYPE);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO false

/* globals */
unsigned ledStartMillis;
unsigned ledNextFlickerMillis;
bool uplinkDataDetected;
bool downlinkDataDetected;
bool ledOn;
uint32_t nextGPSSerialOutput;
uint32_t nextGPSCloudOutput;
unsigned lastMillis;
int state;
int numSend=0;
unsigned dashWakeMillis;
String tempBuffer = "";
String payload = "";
bool foundSMS = false;

void setup()  
{
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Dash.begin();
    Serial2.begin(9600);
    SerialUSB.begin(9600);
    SerialUSB.println("Hologram Dash GPS and Temperature Prototype");
    SerialCloud.begin(115200);

    ledStartMillis = 0; /* LED off by default */  
    ledNextFlickerMillis = 0;
    uplinkDataDetected = false;
    downlinkDataDetected = false;
    ledOn = false;
    ledIndicateData();
    pinMode(RLED, OUTPUT);
    pinMode(GLED, OUTPUT);

    nextGPSSerialOutput = 0;
    nextGPSCloudOutput = 0;
    dashWakeMillis =0;

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    mySerial.begin(9600);
    dht.begin();

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);
    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);

    lastMillis = millis();
    flashLED(GLED,250, 10);
    if (Dash.batteryPercentage() < DASH_BATTERY_THRESHOLD) { //check battery life and blink
          flashLED(RLED, 250, 5);
        }

    SerialCloud.println("Init");
}

void loop() {
    dashSerialGateway(); // opens serial gateway to Dash and debug statements from the Dash modem firmware
    state = handleGPS();
    switch (state) {
      case 11:        
        Serial.println("No Fix");
        alternateLED(GLED, RLED, 125, 10);
        break;
      case 1:
        Serial.println("Serial Print");
        break;
      case 2:
        Serial.println("Hologram Cloud Send");
        Serial.println(numSend);
          flashLED(GLED, 375, 5);
          delay(10000);
          Dash.deepSleepMin(DASH_CLOUD_SLEEP_INTERVAL);
          delay(5000);
          Serial.println("\nWake Up Time: ");
          Serial.println(gpsDateTime());
          dashWakeMillis = millis();
        break;
    }

   // if millis() or timer wraps around, we'll just reset it
   if (lastMillis > millis()) {
       nextGPSSerialOutput = millis();
       nextGPSCloudOutput = millis();
   }
   lastMillis = millis();
}


int handleGPS() {
    int code =0;
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) Serial.print(c);

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
            code = 0;
            return code;  // we can fail to parse a sentence in which case we should just wait for another
        }
    }

    // print out the current stats and send to cloud
    if (nextGPSSerialOutput < millis()) {
        nextGPSSerialOutput = millis() + GPS_SERIAL_SEND_TIME; // reset the timer

        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
        if (GPS.fix) {
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", "); 
            Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
            Serial.print(gpsTempJSON());
            code =1;
        }
        else {
            Serial.println("No GPS Fix");
            code = 11;
        }
        if (Dash.batteryPercentage() < DASH_BATTERY_THRESHOLD) { //check battery life and blink
          flashLED(RLED, 125, 5);
        }
        dhtPrint();
        
    }
    // Send cloud print under any of the following conditions:
    // 1. millis past time for next cloud send
    // 2. Hologram Dash battery level is lower than alert threshold
    // 3. A GPS fix is secured
    // All as long as Dash has been awake longer than MIN_FIRST_SEND_TIME
    // to allow modem to wake in between deep sleep cycles
    if(  ((nextGPSCloudOutput < millis()) || (Dash.batteryPercentage() < DASH_BATTERY_THRESHOLD) 
          ||  (GPS.fix)) && ((millis()-dashWakeMillis) > MIN_FIRST_SEND_TIME) ) {
        nextGPSCloudOutput = millis() + GPS_CLOUD_SEND_TIME; // increment next cloud send time
        SerialCloud.println(gpsTempJSON());
        Serial.println(gpsTempJSON());
        code = 2;
        numSend++;
    }
    return code;
}

void ledIndicateData() {
  ledStartMillis=millis();
  ledOn=true;
}

// flashLED - simple LED helper for signaling state
void flashLED(byte led, int interval, int count) {
  int i = 0;
  while (i < count){
    delay(interval);
    digitalWrite(led, HIGH);
    delay(interval);
    digitalWrite(led, LOW);
    i++;
  }
}

// alternateLED - alternate LED helper for signaling state
void alternateLED(byte l1, byte l2, int interval, int count) {
  int i = 0;
  while (i < count){
    delay(interval);
    digitalWrite(l1, HIGH);
    delay(interval);
    digitalWrite(l1, LOW);
    delay(interval);
    digitalWrite(l2, HIGH);
    delay(interval);
    digitalWrite(l2, LOW);
    i++;
  }
}

// padDateTime: properly pad UTC date time values from satellite date time readings
String padDateTime(int dt) {
  String ret = String(dt);
  if (ret.length() == 1) {
    String i = "0";
    i.concat(ret);
    ret = i;
  }
  return ret;
}

// gpsDateTime - build UTC timestring from GPS date time methods
String gpsDateTime() {
  String dt = "20"; //2012-04-23T18:25:43.511Z
  dt.concat(padDateTime(GPS.year));
  dt.concat("-");
  dt.concat(padDateTime(GPS.month));
  dt.concat("-");
  dt.concat(padDateTime(GPS.day));
  dt.concat("T");
  dt.concat(padDateTime(GPS.hour));
  dt.concat(":");
  dt.concat(padDateTime(GPS.minute));
  dt.concat(":");
  dt.concat(padDateTime(GPS.seconds));
  return dt;  
}

// gpsPrint - Print GPS information to Serial line
void gpsPrint(int fix) {
    Serial.print(gpsDateTime());
    Serial.print("Fix: "); Serial.println((int)GPS.fix);
    if (fix) {
      Serial.print(" Quality: "); Serial.println((int)GPS.fixquality); 
      Serial.print(" Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } 
}

// dhtPrint - print DHT temperature/humiditity info to Serial line
void dhtPrint() {
  Serial.print("Humidity: ");
  Serial.print(dht.readHumidity());
  Serial.println(" %\t");
  Serial.println("Temperature: ");
  Serial.print(dht.readTemperature());
  Serial.print(" *C ");
  Serial.print(dht.readTemperature(true));
  Serial.println(" *F\t");
}

// gpsTempJSON() - Build GPS and temperature JSON object for cloud send
String gpsTempJSON() {
  String lat;
  String lng;
  char coordbuf[16];
  dtostrf(GPS.latitudeDegrees, 0, 6, coordbuf);
  lat.concat(coordbuf);
  dtostrf(GPS.longitudeDegrees, 0, 6, coordbuf);
  lng.concat(coordbuf);
  String ret = "{\"lat\":";
  ret.concat(lat);
  ret.concat(", ");
  ret.concat("\"lng\":");
  ret.concat(lng);
  ret.concat(", ");
  ret.concat("\"fix\":");
  ret.concat((int)GPS.fix);
  ret.concat(", ");
  ret.concat("\"satellites\":");
  ret.concat((int)GPS.satellites);
  ret.concat(", ");
  char buf[8];
  dtostrf(dht.readTemperature(true),4,2,buf);
  ret.concat("\"temp\":");
  ret.concat(buf);
  ret.concat(", ");
  dtostrf(dht.readHumidity(),4,2,buf);
  ret.concat("\"humidity\":");
  ret.concat(buf);
  ret.concat(", ");
  ret.concat("\"time\":\"");
  ret.concat(gpsDateTime());
  ret.concat("\", ");
  ret.concat("\"batt\":");
  ret.concat(Dash.batteryPercentage());
  ret.concat("}");
  return ret;
}

// dashSerialGateway - Opens Hologram Dash as Serial gateway for debug of Dash modem
// events and Serial gateway over USB
void dashSerialGateway() {
  char currChar;
  
  if(ledOn) {
    Dash.onLED();
  } else {
    Dash.offLED();
  }
  if (uplinkDataDetected) {
    /* for uplink data, we turn on LED for duration */
    ledOn=true;
    if(millis() > (ledStartMillis + LED_DATA_INDICATOR_DURATION)) {
      uplinkDataDetected=false;
      ledOn=false;
    }
  }
  if (!uplinkDataDetected && downlinkDataDetected) {
    /* for downlink data, we flash LED for duration */
    if(millis() > ledNextFlickerMillis) {
      if(ledOn) {
        ledOn=false;
        ledNextFlickerMillis = millis() + LED_DATA_INDICATOR_FLICKER_OFF_DURATION;
      } else {
        ledOn=true;
        ledNextFlickerMillis = millis() + LED_DATA_INDICATOR_FLICKER_ON_DURATION;
      }
    }
    if(millis() > (ledStartMillis + LED_DATA_INDICATOR_DURATION)) {
      downlinkDataDetected=false;
      ledOn=false;
    }
  }
  
  while(Serial2.available()) {
    SerialCloud.write(Serial2.read());
    uplinkDataDetected=true;
    ledIndicateData();
  }

  while(SerialUSB.available()) {
    SerialCloud.write(SerialUSB.read());
    uplinkDataDetected=true;
    ledIndicateData();
  }

  while(SerialCloud.available()) {
    currChar = (char)SerialCloud.read();
    SerialUSB.write(currChar);
    Serial2.write(currChar);
    downlinkDataDetected=true;
    ledNextFlickerMillis = millis() + LED_DATA_INDICATOR_FLICKER_ON_DURATION;
    ledIndicateData();
  }
}

// dashReceiveSMS - Parse Dash Modem Serial for SMSRCVD event
string dashReceiveSMS() {
      // check if the current buffer hits the SMSRCVD code.
    if (!foundSMS) {
      if (tempBuffer == "SMSRCVD") {
        foundSMS = true;
      }
    }
    // If it received the SMSRCVD code, the payload will get populated until
    // we get a \n.
    else if (currChar == '\n'){
        SerialUSB.println("\nSMS received: ");
        Serial2.println("\nSMS received: ");
        payload = stripOffLengthNumber(payload);
        SerialUSB.println(payload);
        Serial2.println(payload);
        // DO SOMETHING WITH SMS HERE.
        // reset foundSMS and the payload for the next iteration.
        foundSMS = false;
        payload = "";
    }
    else {
        payload.concat(currChar);
    }
    // Only keep a sliding buffer length of size 7
    // (need to only check for SMSRCVD).
    if (tempBuffer.length() >= 7) {
        tempBuffer.remove(0, 1);
    }
    // add latest char to our buffer.
    tempBuffer.concat(currChar);
  }
}

