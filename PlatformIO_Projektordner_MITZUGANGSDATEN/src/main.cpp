// Libraries have to be imported in the platformio.ini file, e.g. lib_deps =  LMIC-Arduino
#include <Arduino.h>
//Libraries for LoRaWAN
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//Libriaries for TempSensor
#include <OneWire.h>
#include <DallasTemperature.h>
// Secrets (Zugangsdaten TTN usw.)
#include <secrets.h>

//Defining Pins and constants
//#define batteryPin A3 //ändern Pin to read out the battery
//#define KondensatorPin A5
#define PHSensorPin A3
#define Offset 0.00            //deviation compensate PH Sensor
#define TemperatureSensorPin 5 // Pin D5
#define ConductivitySensorPin A4
#define TurbiditySensorPin A2
//weg? #define SAMPLE_SIZE 3
//weg? #define MEASUREMENT_DELAY 20
#define SAMPLE_DELAY 20000
//Creating the Objects, Variables and Constants
//const uint8_t eduKitNumber = 50;
float batSum;
OneWire oneWire(TemperatureSensorPin);
DallasTemperature sensors(&oneWire);
//volatile byte wdt_counter=0;//Counter for Watch Dog

float phValue, tdsValue, temperatureValue, waterFlowValue;
int waterFlowSensorInterrupt = 1;
int waterFlowSensorPin = 3; // Pin 3 = D3
volatile unsigned int waterFlowCount;
uint8_t name = 0; //Loggername
int turbidityValue;
int payloadLength = 11;
int plusbit=6; // +6 Bit = 16 Bit Auflösung 
// The DEVEUI must be in little-endian format (change the order, use www.loratools.nl)
static const u1_t PROGMEM NWKSKEY[16] = SECRET_NWKS_KEY; // LoRaWAN NwkSKey, network session key
static const u1_t PROGMEM APPSKEY[16] = SECRET_APPS_KEY; // Application Session Key
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = SECRET_DEV_ADDR; // <-- Change this address for every node!
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
//osjob used to time the messages (this version uses Deepsleep instead)
static osjob_t sendjob;
const unsigned TX_INTERVAL = 0; //Set to 0 - Deepsleep inseatd!
// Pin mapping for RF95 
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7}
};

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV //GGf meesen und anpassen!
  return result;
}


long oversample(int pin, int pbit) {
// + pbit Bit Auflösung des AD Wandlers
//  
// Matthhias Busse 2.4.2015 Version 1.0
  long summe=0;
  for(int i=0; i < pow(4.0,pbit); i++) {
    summe += analogRead(pin);
  }
  return summe >> pbit;
}

float getPHValue(){
  double vPH;
  double Vcc;
  Vcc = readVcc()/1000.0;
  vPH = Vcc * oversample(PHSensorPin, plusbit) / (1023l << plusbit);
  float pH = 3.5 * vPH + Offset;
  Serial.print("PH V: ");
  Serial.print(vPH);
  Serial.print(" PH value: ");
  Serial.println(pH);
  return pH;
}

float getTemperature(){
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  return temperature;
}

float getTDSValue(){
  double vConductivity;
  double Vcc;
  Vcc = readVcc()/1000.0;
  vConductivity = Vcc * oversample(ConductivitySensorPin, plusbit) / (1023l << plusbit);
  float compensationCoefficient = 1.0 + 0.02 * (temperatureValue - 25.0);                                                                                                                              //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float vConductivityCompensated = vConductivity / compensationCoefficient;                                                                                                                                  //temperature compensation
  float TDS = (133.42 * vConductivityCompensated * vConductivityCompensated * vConductivityCompensated - 255.86 * vConductivityCompensated * vConductivityCompensated + 857.39 * vConductivityCompensated) * 0.5; //convert voltage value to tds value

  Serial.print("Conductivity V: ");
  Serial.print(vConductivity);
  Serial.print(" TDS value: ");
  
  Serial.print(TDS);
  Serial.println(" ppm");

  return TDS;
}

float getTurbidityValue(){
  double vTurbidity;
  int turbidity;
  double Vcc;
  Vcc = readVcc()/1000.0;
  vTurbidity = Vcc * oversample(TurbiditySensorPin, plusbit) / (1023l << plusbit);
  turbidity = -1120.4 * vTurbidity * vTurbidity + 5742.3 * vTurbidity + 4352.9;

  Serial.print("Turbidity V: ");
  Serial.print(vTurbidity);
  Serial.print(" Turbidity value: ");
  Serial.println(turbidity);

  return turbidity;
}

// Interrupt Routine vom Water Flow Sensor
void waterFlowSensorIRRoutine() {
  // Increment the pulse counter ("Umdrehungen")
  //Serial.println("waterFlowSensorIRRoutine");
  waterFlowCount++;
}

// Water Flow Sensor
float getWaterFlowRate(){ //flowSensorPin
  unsigned long oldTime = 0;
  float flowRate = 0.0;
  // The hall-effect flow sensor outputs approximately 4.5 pulses per second per
  // litre/minute of flow.
  float calibrationFactor = 4.5;
  int i = 0;
  while(i < 5){
    if((millis() - oldTime) >= 1000) { //Only process counters once per second
      i++;
      detachInterrupt(waterFlowSensorInterrupt);
      // Water FlowRate in der Sekunde in Liter/Minute  
      flowRate = flowRate + (((1000.0 / (millis() - oldTime)) * waterFlowCount) / calibrationFactor);
      oldTime = millis();

      waterFlowCount = 0;
      attachInterrupt(waterFlowSensorInterrupt, waterFlowSensorIRRoutine, FALLING);
    }
  }

  // debug output
  Serial.print("Flow rate: ");
  Serial.print(flowRate/5.0);
  Serial.println("L/min");
  
  return flowRate/5.0;
}


//Function to get the current voltage of the supply battery
/*float getBatterie() {
  float measuredvbat = analogRead(batteryPin);
  measuredvbat = measuredvbat *  2;    // we divided by 2 with 2 internal 100k resistors attached between batt+ and d9, so multiply back
  measuredvbat = measuredvbat * 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat = measuredvbat / 4.096; // convert to voltage
  return measuredvbat;
}*/

void prepareMeasurements(uint8_t bufferData[]) { 
  turbidityValue = getTurbidityValue();
  phValue = getPHValue(); 
  temperatureValue = getTemperature(); 
  tdsValue = getTDSValue();
  waterFlowValue = getWaterFlowRate();
  
  uint16_t waterPayload = (waterFlowValue * 100);            //0.0 bis 30.0 -> 0 bis 300
  uint16_t tempPayload = (temperatureValue * 100);           //0.00 bis 30.0 -> 0 bis 300
  uint16_t tdsPayload = tdsValue;           //0 bis 1000 
  uint8_t phPayload = (phValue * 10);          //0.0 bis 14.0 -> 0 bis 140
  uint16_t turbidityPayload = turbidityValue;           //0 bis 3000 
  

  //Serial.print("\nwaterPayload: ");
  //Serial.println(waterPayload);
  //Serial.print("tempPayload: ");
  //Serial.println(tempPayload);
  
  int i = 0;
  bufferData[i++] = name;                       //1. Name - 8bit
  bufferData[i++] = highByte(tempPayload);      //2. Temperatur - 16bit
  bufferData[i++] = lowByte(tempPayload);       
  bufferData[i++] = highByte(tdsPayload);       //3. TDS Sensor - 16bit
  bufferData[i++] = lowByte(tdsPayload);
  bufferData[i++] = phPayload;                  //4. PH Sensor - 8bit
  bufferData[i++] = highByte(turbidityPayload); //5. Turbidity Sensor - 16bit
  bufferData[i++] = lowByte(turbidityPayload);
  bufferData[i++] = highByte(waterPayload);     //6. WaterFlow Sesnor - 16bit
  bufferData[i++] = lowByte(waterPayload);
} 

void do_shutdown(){
  // Signal an den Timer, um die Stromzufuhr auszuschalten
  //pinMode(4, OUTPUT); //Pin D4
  digitalWrite(4, HIGH);
  Serial.println("Ausschalten");
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      uint8_t buffer[payloadLength];
      int posBuffer;
      prepareMeasurements(buffer);
      posBuffer = sizeof(buffer)-1;
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, buffer, posBuffer, 0);
      Serial.print("Packet queued. Freq: ");
      Serial.println(LMIC.freq);
      Serial.println("");
      Serial.println("");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setAdrMode(true);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                    Serial.print(F("Received "));
                    Serial.print(LMIC.dataLen);
                    Serial.print(F(" bytes for downlink: 0x"));
                    for (int i = 0; i < LMIC.dataLen; i++) {
                        if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                            Serial.print(F("0"));
                        }
                        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    }
                    
                    Serial.println(F(""));
                    Serial.print(F("RSSI "));
                    Serial.print(LMIC.rssi);
                    Serial.print(F(" SNR "));
                    Serial.print(LMIC.snr / 4);
                    Serial.println(F(""));
                }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            do_shutdown();
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
         default:
            Serial.println((unsigned)ev);
            Serial.println(F("Unknown event"));
            break;
    }
}



void setup() {
  Serial.begin(9600);
  // Interrupt fuer Water Flow Sensor
  attachInterrupt(waterFlowSensorInterrupt, waterFlowSensorIRRoutine, FALLING);
  //LowPower Timer
  pinMode(4, OUTPUT); //Pin D4
  digitalWrite(4, LOW);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
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

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}   