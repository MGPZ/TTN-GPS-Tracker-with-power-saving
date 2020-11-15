#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define LPP_GPS 136   
#include <Adafruit_SleepyDog.h>

static bool devmode = true; //set to true if you want messages output to serial

int OnOffPin = 6; //Specify the pin you have attached to the ON_OFF pin of the ATGM336H module here

static const int RXPin = 4, TXPin = 5; //Specify your GPS RX and TX pins here

static const uint32_t GPSBaud = 9600;
static bool first_run = true;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin); // RX, TX

static const PROGMEM u1_t NWKSKEY[16] = { Your TTN network session key };
static const u1_t PROGMEM APPSKEY[16] = { Your TTN app session key };
static const u4_t DEVADDR = Your TTN device address; // <-- Change this address for every node!
const unsigned TX_INTERVAL = 1;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

  uint8_t coords[11];
  static osjob_t sendjob;
  static osjob_t initjob;
  uint8_t cursor = 0;
  uint8_t channel;

// LoRa module Pin mapping
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 8, 7},
};

  void get_coords () {
  
  if (first_run == false){
  digitalWrite(OnOffPin,LOW);
  if (devmode == true) {
  Serial.println("Pin 6 set to low");
  }
  delay(150);
  if (devmode == true) {
  Serial.println("Going to sleep");
  }
  Serial.flush();
  //Sleep for 1 minute
  int sleepMS1 = Watchdog.sleep(8000); 
  int sleepMS2 = Watchdog.sleep(8000);
  int sleepMS3 = Watchdog.sleep(8000);
  int sleepMS4 = Watchdog.sleep(8000);
  int sleepMS5 = Watchdog.sleep(8000);
  int sleepMS6 = Watchdog.sleep(8000);
  int sleepMS7 = Watchdog.sleep(8000);
  int sleepMS8 = Watchdog.sleep(4000);  }
  else {
  first_run = false;
  }
  digitalWrite(OnOffPin,HIGH);
  if (devmode == true) {
  Serial.println("Pin 6 set to high");
  }
  delay(150); //Give the GPS module a moment to power up
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat,flon,faltitudeGPS,fhdopGPS;
  int fsat;
  fsat = 0;
  unsigned long age;
    // For one second we parse GPS data and report some key values
  if (devmode == true) {
  Serial.print("GPS Starting");
  }
  for (int cycles = 0; cycles <= 300 ; cycles++){ //This prevents the code getting stuck in an infinite loop if the GPS fails to power up properly
  while (fsat < 5){
    
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      if (devmode == true) {
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      }
      if (gps.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }
 
if ( newData ) {
    fsat=gps.satellites.value();
    //fsat = 1;
    flat=gps.location.lat();
    flon=gps.location.lng();
    if (gps.altitude.isValid())
         faltitudeGPS = gps.altitude.meters();
     else
        faltitudeGPS=0;
    fhdopGPS = gps.hdop.value(); 
 }
 
 
  //gps.stats(&chars, &sentences, &failed);


  int32_t lat = flat * 10000;
  int32_t lon = flon * 10000;
  int32_t altitudeGPS = faltitudeGPS  * 100;
  //int16_t altitudeGPS = gps.altitude.meters;
  //int8_t hdopGPS = fhdopGPS;

  channel = 0x01;
  coords[0] = channel;
  coords[1] = LPP_GPS;

  coords[2] = (lat >> 16);
  coords[3] = (lat >> 8);
  coords[4] = lat;

  coords[5] = (lon >> 16);
  coords[6] = (lon >> 8);
  coords[7] = lon;

  coords[8] = (altitudeGPS >> 16);
  coords[9] = (altitudeGPS >> 8);
  coords[10] = altitudeGPS;
  //fsat = 1;
  if (devmode == true) {
  Serial.print("Sats = ");
  Serial.println(fsat);
  Serial.print("Cycles = ");
  Serial.println(cycles);
    }
   }
  }
 }

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    if (devmode == true) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    }
  } else {
    // Prepare upstream data transmission at the next possible time.

    get_coords();
    LMIC_setTxData2(1, (uint8_t*) coords, sizeof(coords), 0);
    
  }
}

  // Next TX is scheduled after TX_COMPLETE event.

void onEvent (ev_t ev) {
  if (devmode == true) {
  Serial.print(os_getTime());
  Serial.print(": ");
  }
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      if (devmode == true) {
      Serial.println(F("EV_SCAN_TIMEOUT"));
      }
      break;
    case EV_BEACON_FOUND:
    if (devmode == true) {
      Serial.println(F("EV_BEACON_FOUND"));
    }
      break;
    case EV_BEACON_MISSED:
    if (devmode == true) {
      Serial.println(F("EV_BEACON_MISSED"));
    }
      break;
    case EV_BEACON_TRACKED:
     if (devmode == true) {
      Serial.println(F("EV_BEACON_TRACKED"));
     }
      break;
    case EV_JOINING:
      if (devmode == true) {
      Serial.println(F("EV_JOINING"));
      }
      break;
    case EV_JOINED:
      if (devmode == true) {
      Serial.println(F("EV_JOINED"));
      }
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      if (devmode == true) {
      Serial.println(F("EV_RFU1"));
      }
      break;
    case EV_JOIN_FAILED:
      if (devmode == true) {
      Serial.println(F("EV_JOIN_FAILED"));
      }
      break;
    case EV_REJOIN_FAILED:
      if (devmode == true) {
      Serial.println(F("EV_REJOIN_FAILED"));
      }
      break;
      break;
    case EV_TXCOMPLETE:
      if (devmode == true) {
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      }
      if (LMIC.txrxFlags & TXRX_ACK)
        if (devmode == true) {
        Serial.println(F("Received ack"));
        }
      if (LMIC.dataLen) {
        if (devmode == true) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        }
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      if (devmode == true) {
      Serial.println(F("EV_LOST_TSYNC"));
      }
      break;
    case EV_RESET:
      if (devmode == true) {
      Serial.println(F("EV_RESET"));
      }
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      if (devmode == true) {
      Serial.println(F("EV_RXCOMPLETE"));
      }
      break;
    case EV_LINK_DEAD:
      if (devmode == true) {
      Serial.println(F("EV_LINK_DEAD"));
      }
      break;
    case EV_LINK_ALIVE:
      if (devmode == true) {
      Serial.println(F("EV_LINK_ALIVE"));
      }
      break;
    default:
      if (devmode == true) {
      Serial.println(F("Unknown event"));
      }
      break;

  }
}

void setup()
{
  Serial.begin(115200);
  if (devmode == true) {
  Serial.println(F("Starting"));
  }
  ss.begin(GPSBaud);
  pinMode(OnOffPin, OUTPUT);
  static bool first_run = true;
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
     
}

void loop() {
  //Serial.println("Looping");
  os_runloop_once();
}



  
