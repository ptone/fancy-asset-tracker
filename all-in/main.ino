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


bool debug = true;
int lastSecond = 0;
bool ledState = false;

int trackerMode = 0;
// 0 uninitialized (implicit)
// 1 setup (implicit)
// 2 acquiring GPS
// 3 active - watch accelerometer
// 4 sleeping (implicit)
// 5 low battery mode

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);


STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));


unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
time_t lastIdleCheckin = 0;

/* int lastKnownClockHour = 0; */

unsigned long lastGPSPoll = 0;
// TODO is it make any different slow or fast? // 3000
unsigned long GPSPollInterval = 500;
bool hasMotion = false;
bool inSleep = false;

#define PUBLISH_DELAY (60 * 1000)

// if no motion for 3 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (3 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)


void setup() {
    debugPrint("setup");
    lastMotion = 0;
    lastPublish = 0;

    initAccel();

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

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
}



void loop() {
    unsigned long now = millis();
    switch (trackerMode) {
        case 2: //acquiring GPS
            debugPrint("case 2: acquiring GPS");
            if (inSleep) {
                debugPrint("waking!");
                // reenable the GPS
                digitalWrite(D6, HIGH);
                // avoid going right back to sleep
                lastMotion = now;
                inSleep = false;
            }
            // only read GPS every<GPSPollInterval> seconds
            debugPrint(String::format("now: %lu, lastPoll:%lu", now, lastGPSPoll));
            if (now > (lastGPSPoll + GPSPollInterval)) {
                lastGPSPoll = now;
                blink(1);
                checkGPS();
                bool lowBatt = fuel.getSoC() < .15;
                if (GPS.latitude != 0) {
                    debugPrint("have GPS signal");
                    Particle.connect();
                    publishGPS();
                    trackerMode = 3;
                    // over-ride and set low power mode if battery low
                    if (lowBatt) { trackerMode = 5; }
                    break;
                }
                // no GPS after 2 minutes
                // TODO return to shorter value - reduced for testing
                /* if (millis() > 120000) { */

                if (millis() > 40000) {
                    blink(5);
                    // been a while and still no connect
                    if (Particle.connected() == false) {
                        Particle.connect();
                        debugPrint("connecting");
                        // clock syncs here
                    }
                    for (int i = 0; i < 70; i++) {
                        if (Cellular.ready()) {
                            debugPrint("Cellular ready");
                            break;
                        }
                        debugPrint("wait cell ready");
                        delay(1000);
                    }
                    debugPrint("sending GPS lock failure");
                    Particle.publish("S", "F");
                    delay(3000);
                    inSleep = true;
                    accel.setClick(0, CLICKTHRESHHOLD);
                    // TODO - even more in lowbattery mode?
                    // TODO make this at least 5min - reduced for testing
                    System.sleep(SLEEP_MODE_DEEP, (1 * 60));
                }
            }
            break;
        case 3: //active mode, watch accelerometer
            debugPrint("case 3");
            if (lastIdleCheckin == 0) {
                lastIdleCheckin = Time.now();
            }

            if (lastMotion > now) { lastMotion = now; }
            if (lastPublish > now) { lastPublish = now; }

            // we'll be woken by motion, lets keep listening for more motion.
            // if we get two in a row, then we'll connect to the internet and start reporting in.
            hasMotion = digitalRead(WKP);


            if (hasMotion) {
                debugPrint("BUMP");
                lastMotion = now;

                if (Particle.connected() == false) {
                    debugPrint("CONNECTING DUE TO MOTION!");
                    Particle.connect();
                }
            }

            // use the real-time-clock here, instead of millis.
            if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {

                // it's been too long!  Lets say hey!
                if (Particle.connected() == false) {
                    debugPrint("CONNECTING DUE TO IDLE!");
                    Particle.connect();
                }

                // TODO consider dropping  - GPS sent on wake
                Particle.publish("S", "I");
                lastIdleCheckin = now;
            }


            // have we published recently?
            debugPrint("lastPublish is " + String(lastPublish));
            if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
                blink(3);
                lastPublish = millis();
                checkGPS();
                publishGPS();
            }

            // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
            // accidentally idle out.
            if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
                // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
                // if the accel triggers an interrupt, we'll wakeup earlier than that.
                Serial.println(now);
                debugPrint("sleeping");
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
                accel.setClick(1, CLICKTHRESHHOLD);
                System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
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
            accel.setClick(0, CLICKTHRESHHOLD);
            System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
    }
    delay(100);
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


void checkGPS() {
    // process and dump everything from the module through the library.
    debugPrint("check GPS");
    while (mySerial.available()) {
        char c = GPS.read();

        // lets echo the GPS output until we get a good clock reading, then lets calm things down.
        //if (!hasGPSTime) {
        //   Serial.print(c);
        //}
        Serial.print(c);

        if (GPS.newNMEAreceived()) {
            debugPrint("NMEA received");
            GPS.parse(GPS.lastNMEA());
            debugPrint("GPS Latitude: ");
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
        + ",\"a\":"     + String(GPS.altitude)
        + ",\"q\":"     + String(GPS.fixquality)
        + ",\"spd\":"   + String(GPS.speed)
        + ",\"mot\":"   + String(motionInTheLastMinute)
        + ",\"s\": "  + String(GPS.satellites)
        + ",\"vcc\":"   + String(fuel.getVCell())
        + ",\"soc\":"   + String(fuel.getSoC())
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

void debugPrint(String msg) {
    if (debug) {
        Serial.println(msg);
    }
}
