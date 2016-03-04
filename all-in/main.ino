#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"

#include <math.h>
#include "math.h"
#include <ctype.h>


#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);

FuelGauge fuel;

#define MY_NAME "AssetTracker"

#define CLICKTHRESHHOLD 20

#define PUBLISH_DELAY (60 * 1000)

// how long to try and fix the GPS
#define GPS_FIX_TIME (2 * 60 * 1000)

// if no motion for 3 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (3 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)

#define GPS_POLL_INTERVAL 1000

#define BUILD_VERSION 34



bool debug = false;
bool GPSFixedOnce = false;
unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
int lastIdleCheckin = 0;
unsigned long lastGPSPoll = 0;
bool hasMotion = false;
bool inSleep = false;


int trackerMode = 0;
// 0 uninitialized (implicit)
// 1 setup (implicit)
// 2 acquiring GPS
// 3 active - watch accelerometer
// 4 sleeping (implicit)
// 5 low battery mode
// 6 charging

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

void setup() {
    dPrint("setup");
    lastMotion = 0;
    lastPublish = 0;

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);

    initAccel();

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);


    // used to trigger "charge mode"
    pinMode(D2, INPUT_PULLUP);

    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);


    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);


    // request everything!
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    delay(250);

    // turn off antenna updates
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);
    trackerMode = 2;

    //setup first fix
    Particle.connect();
    waitCellReady();
    delay(1000);
    //googleplex location
    //37.4184,-122.0880,100
    String locationString = "34.447847,-119.730957,200";
    time_t timeValue = Time.now();
    String timeString = Time.format(timeValue, "%Y,%m,%d,%H,%M,%S");
    //  PMTK740,YYYY,MM,DD,hh,mm,ss*CS<CR><LF>
    //  PMTK741,Lat,Long,Alt,YYYY,MM,DD,hh,mm,ss *CS<CR><LF>
    //The packet contains reference location for the GPS receiver. To have faster TTFF, the accuracy of the location shall be better than 30km.
    String gpsTimeCmd = "PMTK740," + timeString;
    String locationTimeCmd = "PMTK741,"+locationString + "," + timeString;


    String cmd = String::format("$%s*%02x", locationTimeCmd.c_str(), crc8(locationTimeCmd));
    mySerial.println(cmd);
    //GPS.sendCommand(cmd.c_str());     // why doesn't this support const char *...
    delay(250);
    Cellular.off();

    //setup done
    digitalWrite(D7, LOW);
}


void loop() {
    // note millis resets to 0 on every wake
    unsigned long now = millis();
    dPrint(String::format("now: %lu", now));
    //ensure that we pull a fresh latitude
    GPS.latitude = 0.0;
    digitalWrite(D7, LOW);

    if (digitalRead(D2) == 0) {
        // pin D2 bridged to ground signals charging mode
        // no connections should be made
        trackerMode = 6;
    } else {
        if (trackerMode == 6) {
            // jumper just unplugged - go into acquisition mode
            trackerMode = 2;
        }
    }

    switch (trackerMode) {
        case 2: //acquiring GPS
            dPrint("case 2: acquiring GPS");
            if (inSleep) {
                dPrint("waking!");
                // avoid going right back to sleep
                lastMotion = now;
                inSleep = false;
            }
            // only read GPS every<GPS_POLL_INTERVAL> seconds
            dPrint(String::format("now: %lu, lastPoll:%lu", now, lastGPSPoll));
            if (now > (lastGPSPoll + GPS_POLL_INTERVAL)) {
                lastGPSPoll = now;
                blink(1);
                checkGPS();
                bool lowBatt = fuel.getSoC() < .25;
                if (GPS.latitude != 0) {
                    dPrint("have GPS signal");
                    Particle.connect();
                    waitCellReady();
                    publishGPS();
                    if (GPSFixedOnce == false) {
                        GPSFixedOnce = true;
                        Particle.publish("S", String::format("C%lu", now / 1000));
                    } else {
                        Particle.publish("S", String::format("c%lu", now / 1000));
                    }
                    trackerMode = 3;
                    // over-ride and set low power mode if battery low
                    if (lowBatt) { trackerMode = 5; }
                    break;
                }
                // no GPS after given time
                // TODO if millis is retain on sleep properly - this fires too soon
                if ((debug || GPSFixedOnce) && (millis() > GPS_FIX_TIME)) {
                    //don't sleep unless we've had one GPS fix ever
                    blink(5);
                    // been a while and still no connect
                    if (Particle.connected() == false) {
                        Particle.connect();
                        dPrint("connecting");
                        // clock syncs here
                    }
                    waitCellReady();
                    dPrint("sending GPS lock failure");
                    Particle.publish("S", "F");
                    delay(3000);
                    inSleep = true;
                    //accel.setClick(0, CLICKTHRESHHOLD);
                    // TODO - even more in lowbattery mode?
                    if (debug) {
                        System.sleep(SLEEP_MODE_DEEP, (60));
                    } else {
                        System.sleep(SLEEP_MODE_DEEP, (5 * 60));
                    }
                }
            }
            break;
        case 3: //active mode, watch accelerometer
            dPrint("case 3");
            if (lastIdleCheckin == 0) {
                lastIdleCheckin = Time.now();
            }

            if (lastMotion > now) { lastMotion = now; }
            if (lastPublish > now) { lastPublish = now; }

            // we'll be woken by motion, lets keep listening for more motion.
            // if we get two in a row, then we'll connect to the internet and start reporting in.
            hasMotion = digitalRead(WKP);


            if (hasMotion) {
                dPrint("BUMP");
                lastMotion = now;

                if (Particle.connected() == false) {
                    dPrint("CONNECTING DUE TO MOTION!");
                    Particle.connect();
                }
            }

            // use the real-time-clock here, instead of millis.
            if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {

                // it's been too long!  Lets say hey!
                if (Particle.connected() == false) {
                    dPrint("CONNECTING DUE TO IDLE!");
                    Particle.connect();
                }

                // TODO consider dropping  - GPS sent on wake
                Particle.publish("S", "I" + String(Time.now()));
                lastIdleCheckin = Time.now();
            }


            // have we published recently?
            dPrint("lastPublish is " + String(lastPublish));
            if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
                blink(3);
                checkGPS();
                if (GPS.latitude != 0.0) {
                    publishGPS();
                    lastPublish = millis();
                }
            }

            // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
            // accidentally idle out.
            if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
                // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
                // if the accel triggers an interrupt, we'll wakeup earlier than that.
                Serial.println(now);
                dPrint("sleeping");
                //TODO see if works better if only set in setup
                //accel.setClick(1, CLICKTHRESHHOLD);
                Particle.publish("S", "S");

                lastPublish = 0;
                lastMotion = 0;

                // Hey GPS, please stop using power, kthx.
                digitalWrite(D6, HIGH);

                // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
                delay(10*1000);

                // turn off particle radio
                Particle.disconnect();

                // wake in GPS acquisition mode
                trackerMode = 2;
                blink(4);
                inSleep = true;
                /* System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP); */
                System.sleep(WKP, CHANGE, HOW_LONG_SHOULD_WE_SLEEP);
                /* System.sleep(uint16_t wakeUpPin, uint16_t edgeTriggerMode, long seconds) */
                /* System.sleep(WKP, CHANGE, HOW_LONG_SHOULD_WE_SLEEP); */
            }
            break;
        case 5: //low battery
            // we woke in GPS mode - so some status was sent already

            // Hey GPS, please stop using power, kthx.
            digitalWrite(D6, HIGH);

            // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
            delay(5*1000);
            // turn off particle radio
            Particle.disconnect();

            // wake in GPS acquisition mode
            trackerMode = 2;
            // go into true deep sleep - this should NOT wake on motion
            blink(4);
            inSleep = true;
            //TODO this should be safe, but debugging some overly sleepy
            accel.setClick(0, CLICKTHRESHHOLD);
            delay(5000);
            System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
        case 6: // charge mode
            if (Particle.connected() == true) {
                Particle.disconnect();
            }
            digitalWrite(D7, HIGH);
            delay(3000);
    }
    if (debug){
        //lets not flood the serial console...
        delay(1000);
    } else {
        delay(10);
    }
}


void initAccel() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 4 gravities range
    accel.setRange(LIS3DH_RANGE_4_G);

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks

    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}


bool waitCellReady() {
    // note, in theory particle.connect is blocking, but ....
    for (int i = 0; i < 70; i++) {
        if (Cellular.ready()) {
            dPrint("Cellular ready");
            return true;
        }
        dPrint("wait cell ready");
        delay(1000);
    }
    dPrint("Failure to establish cell connection");
    return false;
}

void checkGPS() {
    // process and dump everything from the module through the library.
    dPrint("check GPS");
    while (mySerial.available()) {
        char c = GPS.read();
        Serial.print(c);

        if (GPS.newNMEAreceived()) {
            dPrint("NMEA received");
            GPS.parse(GPS.lastNMEA());
            dPrint("GPS Latitude: ");
            Serial.println(GPS.latitude);
        }
    }
}

void publishGPS() {
    unsigned int msSinceLastMotion = (millis() - lastMotion);
    int motionInTheLastMinute = (msSinceLastMotion < 60000);

    /*
    String gps_line = String::format("%f,%f,%f,%f,
        convertDegMinToDecDeg(GPS.latitude), convertDegMinToDecDeg(GPS.longitude), GPS.altitude, GPS.speed);
    */

    String gps_line =
          "{\"lat\":"    + String(convertDegMinToDecDeg(GPS.latitude))
        + ",\"lon\":-"   + String(convertDegMinToDecDeg(GPS.longitude))
        /* + ",\"a\":"      + String(GPS.altitude) */
        /* + ",\"q\":"      + String(GPS.fixquality) */
        + ",\"spd\":"    + String(GPS.speed)
        /* + ",\"mot\":"    + String(motionInTheLastMinute) */
        /* + ",\"s\": "     + String(GPS.satellites) */
        /* + ",\"vcc\":"    + String(fuel.getVCell()) */
        + ",\"soc\":"    + String(fuel.getSoC())
        + ",\"v\":"      + String(BUILD_VERSION)
        + "}";

    blink(2);
    Particle.publish("G", gps_line, 60, PRIVATE);
}

void blink(int ct) {
    for(int i = 0; i < ct; i++) {
        digitalWrite(D7, HIGH);
        delay(50);
        digitalWrite(D7, LOW);
        delay(100);
    }
}

void dPrint(String msg) {
    if (debug) {
        Serial.println(msg);
    }
}


int crc8(String str) {
  int len = str.length();
  const char * buffer = str.c_str();

  int crc = 0;
  for(int i=0;i<len;i++) {
    crc ^= (buffer[i] & 0xff);
  }
  return crc;
}
