///////////////////////////////////////////////////////////////////////////////
// Bresser_WeatherStation_CC1101_TTN.ino
// 
// Bresser 5-in-1/6-in-1 868 MHz Weather Sensor Radio Receiver based on CC1101, 
// ESP8266/ESP32 and RFM95W - 
// provides data via LoRaWAN to The Things Network
//
// Based on:
// Bresser5in1-CC1101 by Sean Siford (https://github.com/seaniefs/Bresser5in1-CC1101)
// https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_6in1.c
// RadioLib by Jan Gromeš (https://github.com/jgromes/RadioLib)
// MCCI LoRaWAN LMIC library by Thomas Telkamp and Matthijs Kooijman / Terry Moore, MCCI
// Lora-Serialization by Joscha Feth (with modifications)
//
//
// created: 03/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
// History:
//
// 20220309 Created
// 20220418 Added status output pins SENSOR_DECOK and TTN_TXCOMPL
//
// ToDo:
// 
// - Check CC1101 receive buffer behaviour! Is it flushed after reading or
//   does it overflow? Make sure that always fresh data is read.
//
// Notes:
//
// - Keep some distance between the antennas of the RFM95 module (TX/RX)
//   and the CC1101 (RX) module!!! Otherwise the CC1101 could be destroyed,
//   because both use the same frequency.
//
///////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 * 
 * You will need to also install the library github.com/PaulStoffregen/Time;
 * you need a version that has TimeLib.h.
 *
 *******************************************************************************/

#define BRESSER_6_IN_1
#define RX_RETRIES 5
#define RADIOLIB_DEBUG
//#define TTN_DEBUG
//#define CC1101_DEBUG
#define _DEBUG_MODE_
#include <Arduino.h>
#include <RadioLib.h>

// requires library: github.com/PaulStoffregen/Time 
// -> Library Manager: "Time" by Michael Margolis
#include <TimeLib.h>    // can't use <Time.h> starting with v1.6.1

// MCCI LoRaWAN LMIC library
#include <arduino_lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// Modified "Lora Serialization" by Joscha Feth
#include "src/LoRa_Serialization/src/LoraMessage.h"

// APPEUI, DEVEUI and APPKEY
#include "secrets.h"

#ifndef SECRET
    // The following constants should be copied to secrets.h and configured appropriately
    // according to the settings from TTN Console

    // This EUI must be in little-endian format, so least-significant-byte
    // first. When copying an EUI from ttnctl output, this means to reverse
    // the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
    // 0x70.
    static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // This should also be in little endian format, see above.
    static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // This key should be in big endian format (or, since it is not really a
    // number but a block of memory, endianness does not really apply). In
    // practice, a key taken from ttnctl can be copied as-is.
    static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif


// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
// Please respect the TTN Fair Use Policy!
const unsigned TX_INTERVAL  = 60;

// TX message payload size
const uint8_t PAYLOAD_SIZE = 24;

// Pin config
#define PIN_CC1101_CS     15
#define PIN_CC1101_GDO0   26
#define PIN_CC1101_GDO2   4
#define PIN_LMIC_NSS      27
#define PIN_LMIC_RST      32
#define PIN_LMIC_DIO0     21
#define PIN_LMIC_DIO1     33
#define PIN_LMIC_DIO2     25
#define PIN_SENSOR_DECOK  12
#define PIN_TTN_TXCOMPL   13

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// TX payload buffer
static uint8_t loraData[PAYLOAD_SIZE];


// RX payload struct / type
struct rx_payload_s {
    bool       valid;
};

typedef struct rx_payload_s rx_payload_t;

// Weather data struct / type
struct WeatherData_S {
    uint8_t  s_type;               // only 6-in1
    uint32_t sensor_id;            // 5-in-1: 1 byte / 6-in-1: 4 bytes
    uint8_t  chan;                 // only 6-in-1
    bool     temp_ok;              // only 6-in-1
    float    temp_c;
    int      humidity;
    bool     uv_ok;                // only 6-in-1
    float    uv;                   // only 6-in-1
    bool     wind_ok;              // only 6-in-1
    float    wind_direction_deg;
    float    wind_gust_meter_sec;
    float    wind_avg_meter_sec;
    bool     rain_ok;              // only 6-in-1
    float    rain_mm;
    bool     battery_ok;
    bool     moisture_ok;          // only 6-in-1
    int      moisture;             // only 6-in-1
};

typedef struct WeatherData_S WeatherData_t;

// Weather data object
WeatherData_t weatherData;

// RX payload object
rx_payload_t rx_payload;

// sendjob object
static osjob_t sendjob;


// Pin mapping ESP32
// SPI2 is used on ESP32 per default! (e.g. see https://github.com/espressif/arduino-esp32/tree/master/variants/doitESP32devkitV1)
const lmic_pinmap lmic_pins = {
    .nss = PIN_LMIC_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PIN_LMIC_RST,
    .dio = {PIN_LMIC_DIO0, PIN_LMIC_DIO1, PIN_LMIC_DIO2}
};


// Generate CC1101 radio module instance
CC1101 radio = new Module(PIN_CC1101_CS, PIN_CC1101_GDO0, RADIOLIB_NC, PIN_CC1101_GDO2);

typedef enum DecodeStatus {
    DECODE_OK, DECODE_PAR_ERR, DECODE_CHK_ERR, DECODE_DIG_ERR
} DecodeStatus;

// Prototypes
DecodeStatus decodeBresser5In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *pOut);
DecodeStatus decodeBresser6In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *pOut);
bool genWeatherdata(WeatherData_t *pOut);

//
// RX message decoding stub
//
void decode_rx_message(uint8_t* frame, uint8_t offs, uint8_t len, struct rx_payload_s * payload) {
     payload->valid = true;
}

//
// Hex printing function
//
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

//
// LMIC Event Handler
//
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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
              Serial.print("RSSI: ");
              Serial.println(LMIC.rssi);
              Serial.print("SNR: ");
              Serial.println(LMIC.snr);
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	          // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            digitalWrite(PIN_TTN_TXCOMPL, HIGH);
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              for (uint16_t i=0; i<LMIC.dataLen; i++) {
                 printHex2(LMIC.frame[LMIC.dataBeg + i]);
              }
              Serial.println();
              decode_rx_message(LMIC.frame, LMIC.dataBeg, LMIC.dataLen, &rx_payload);
              Serial.print("valid: ");
              Serial.println(rx_payload.valid);
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


//
// Send data to The Things Network
//
void do_send(osjob_t* j) {
    bool decode_ok = false;
  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        #ifndef TTN_DEBUG
            for (uint8_t i=0; i<RX_RETRIES; i++) {
                Serial.print(F("getWeatherdata()..."));
                decode_ok = getWeatherdata();
                if (decode_ok)
                    break;
            }
        #else
            decode_ok = genWeatherdata(&weatherData);
        #endif
        if (decode_ok) {
            Serial.println(F("o.k."));
            digitalWrite(PIN_SENSOR_DECOK, HIGH);
        } else {
            Serial.println(F("failed."));
            digitalWrite(PIN_SENSOR_DECOK, LOW);
        }
        // FIXME: Send only new (valid) data, otherwise schedule next transmission
        LoraEncoder encoder(loraData);
        encoder.writeUint32(weatherData.sensor_id);
        encoder.writeBitmap(false, false, false, false, false, false, false, weatherData.battery_ok);
        encoder.writeTemperature(weatherData.temp_c);
        encoder.writeUint8(weatherData.humidity);
        encoder.writeRawFloat(weatherData.wind_gust_meter_sec);
        encoder.writeRawFloat(weatherData.wind_avg_meter_sec);
        encoder.writeRawFloat(weatherData.wind_direction_deg);
        encoder.writeRawFloat(weatherData.rain_mm);
        //encoder.writeRawFloat(radio.getRSSI()); // FIXME: Integer should be sufficient
        //encoder.writeUint8(radio.getLQI());

        Serial.println("Sending LoRa Data...");
        // Optionally set the Frame Port (1 to 255)
        // uint8_t framePort = 1;
        // lora.sendData(loraData, sizeof(loraData), lora.frameCounter, framePort);
        //Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
        //lora.frameCounter++;

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, loraData, sizeof(loraData), 0);
        digitalWrite(PIN_TTN_TXCOMPL, LOW);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/util.c
//
uint16_t lfsr_digest16(uint8_t const message[], unsigned bytes, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k) {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i) {
            // fprintf(stderr, "key at bit %d : %04x\n", i, key);
            // if data bit is set then xor with key
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}


//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/util.c
//
int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}


// Cribbed from rtl_433 project - but added extra checksum to verify uu
//
// Example input data:
//   EA EC 7F EB 5F EE EF FA FE 76 BB FA FF 15 13 80 14 A0 11 10 05 01 89 44 05 00
//   CC CC CC CC CC CC CC CC CC CC CC CC CC uu II SS GG DG WW  W TT  T HH RR  R Bt
// - C = Check, inverted data of 13 byte further
// - uu = checksum (number/count of set bits within bytes 14-25)
// - I = station ID (maybe)
// - G = wind gust in 1/10 m/s, normal binary coded, GGxG = 0x76D1 => 0x0176 = 256 + 118 = 374 => 37.4 m/s.  MSB is out of sequence.
// - D = wind direction 0..F = N..NNE..E..S..W..NNW
// - W = wind speed in 1/10 m/s, BCD coded, WWxW = 0x7512 => 0x0275 = 275 => 27.5 m/s. MSB is out of sequence.
// - T = temperature in 1/10 °C, BCD coded, TTxT = 1203 => 31.2 °C
// - t = temperature sign, minus if unequal 0
// - H = humidity in percent, BCD coded, HH = 23 => 23 %
// - R = rain in mm, BCD coded, RRxR = 1203 => 31.2 mm
// - B = Battery. 0=Ok, 8=Low.
// - S = sensor type, only low nibble used, 0x9 for Bresser Professional Rain Gauge
//
// Parameters:
//
// msg     - Pointer to message
// msgSize - Size of message
// pOut    - Pointer to WeatherData
//
// Returns:
//
// DECODE_OK      - OK - WeatherData will contain the updated information
// DECODE_PAR_ERR - Parity Error
// DECODE_CHK_ERR - Checksum Error
//
DecodeStatus decodeBresser5In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *pOut) { 
    // First 13 bytes need to match inverse of last 13 bytes
    for (unsigned col = 0; col < msgSize / 2; ++col) {
        if ((msg[col] ^ msg[col + 13]) != 0xff) {
            Serial.printf("%s: Parity wrong at %u\n", __func__, col);
            return DECODE_PAR_ERR;
        }
    }

    // Verify checksum (number number bits set in bytes 14-25)
    uint8_t bitsSet = 0;
    uint8_t expectedBitsSet = msg[13];

    for(uint8_t p = 14 ; p < msgSize ; p++) {
      uint8_t currentByte = msg[p];
      while(currentByte) {
        bitsSet += (currentByte & 1);
        currentByte >>= 1;
      }
    }

    if (bitsSet != expectedBitsSet) {
       Serial.printf("%s: Checksum wrong actual [%02X] != expected [%02X]\n", __func__, bitsSet, expectedBitsSet);
       return DECODE_CHK_ERR;
    }

    pOut->sensor_id = msg[14];

    int temp_raw = (msg[20] & 0x0f) + ((msg[20] & 0xf0) >> 4) * 10 + (msg[21] &0x0f) * 100;
    if (msg[25] & 0x0f) {
        temp_raw = -temp_raw;
    }
    pOut->temp_c = temp_raw * 0.1f;

    pOut->humidity = (msg[22] & 0x0f) + ((msg[22] & 0xf0) >> 4) * 10;

    pOut->wind_direction_deg = ((msg[17] & 0xf0) >> 4) * 22.5f;

    int gust_raw = ((msg[17] & 0x0f) << 8) + msg[16];
    pOut->wind_gust_meter_sec = gust_raw * 0.1f;

    int wind_raw = (msg[18] & 0x0f) + ((msg[18] & 0xf0) >> 4) * 10 + (msg[19] & 0x0f) * 100;
    pOut->wind_avg_meter_sec = wind_raw * 0.1f;

    int rain_raw = (msg[23] & 0x0f) + ((msg[23] & 0xf0) >> 4) * 10 + (msg[24] & 0x0f) * 100;
    pOut->rain_mm = rain_raw * 0.1f;

    pOut->battery_ok = (msg[25] & 0x80) ? false : true;

    return DECODE_OK;
}


//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_6in1.c
//
/**
Decoder for Bresser Weather Center 6-in-1.
- also Bresser Weather Center 7-in-1 indoor sensor.
- also Bresser new 5-in-1 sensors.
- also Froggit WH6000 sensors.
- also rebranded as Ventus C8488A (W835)
- also Bresser 3-in-1 Professional Wind Gauge / Anemometer PN 7002531
There are at least two different message types:
- 24 seconds interval for temperature, hum, uv and rain (alternating messages)
- 12 seconds interval for wind data (every message)
Also Bresser Explore Scientific SM60020 Soil moisture Sensor.
https://www.bresser.de/en/Weather-Time/Accessories/EXPLORE-SCIENTIFIC-Soil-Moisture-and-Soil-Temperature-Sensor.html
Moisture:
    f16e 187000e34 7 ffffff0000 252 2 16 fff 004 000 [25,2, 99%, CH 7]
    DIGEST:8h8h ID?8h8h8h8h FLAGS:4h BATT:1b CH:3d 8h 8h8h 8h8h TEMP:12h 4h MOIST:8h TRAILER:8h8h8h8h4h
Moisture is transmitted in the humidity field as index 1-16: 0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99.
{206}55555555545ba83e803100058631ff11fe6611ffffffff01cc00 [Hum 96% Temp 3.8 C Wind 0.7 m/s]
{205}55555555545ba999263100058631fffffe66d006092bffe0cff8 [Hum 95% Temp 3.0 C Wind 0.0 m/s]
{199}55555555545ba840523100058631ff77fe668000495fff0bbe [Hum 95% Temp 3.0 C Wind 0.4 m/s]
{205}55555555545ba94d063100058631fffffe665006092bffe14ff8
{206}55555555545ba860703100058631fffffe6651ffffffff0135fc [Hum 95% Temp 3.0 C Wind 0.0 m/s]
{205}55555555545ba924d23100058631ff99fe68b004e92dffe073f8 [Hum 96% Temp 2.7 C Wind 0.4 m/s]
{202}55555555545ba813403100058631ff77fe6810050929ffe1180 [Hum 94% Temp 2.8 C Wind 0.4 m/s]
{205}55555555545ba98be83100058631fffffe6130050929ffe17800 [Hum 95% Temp 2.8 C Wind 0.8 m/s]
                                          TEMP  HUM
2dd4  1f 40 18 80 02 c3 18 ff 88 ff 33 08 ff ff ff ff 80 e6 00 [Hum 96% Temp 3.8 C Wind 0.7 m/s]
2dd4  cc 93 18 80 02 c3 18 ff ff ff 33 68 03 04 95 ff f0 67 3f [Hum 95% Temp 3.0 C Wind 0.0 m/s]
2dd4  20 29 18 80 02 c3 18 ff bb ff 33 40 00 24 af ff 85 df    [Hum 95% Temp 3.0 C Wind 0.4 m/s]
2dd4  a6 83 18 80 02 c3 18 ff ff ff 33 28 03 04 95 ff f0 a7 3f
2dd4  30 38 18 80 02 c3 18 ff ff ff 33 28 ff ff ff ff 80 9a 7f [Hum 95% Temp 3.0 C Wind 0.0 m/s]
2dd4  92 69 18 80 02 c3 18 ff cc ff 34 58 02 74 96 ff f0 39 3f [Hum 96% Temp 2.7 C Wind 0.4 m/s]
2dd4  09 a0 18 80 02 c3 18 ff bb ff 34 08 02 84 94 ff f0 8c 0  [Hum 94% Temp 2.8 C Wind 0.4 m/s]
2dd4  c5 f4 18 80 02 c3 18 ff ff ff 30 98 02 84 94 ff f0 bc 00 [Hum 95% Temp 2.8 C Wind 0.8 m/s]
{147} 5e aa 18 80 02 c3 18 fa 8f fb 27 68 11 84 81 ff f0 72 00 [Temp 11.8 C  Hum 81%]
{149} ae d1 18 80 02 c3 18 fa 8d fb 26 78 ff ff ff fe 02 db f0
{150} f8 2e 18 80 02 c3 18 fc c6 fd 26 38 11 84 81 ff f0 68 00 [Temp 11.8 C  Hum 81%]
{149} c4 7d 18 80 02 c3 18 fc 78 fd 29 28 ff ff ff fe 03 97 f0
{149} 28 1e 18 80 02 c3 18 fb b7 fc 26 58 ff ff ff fe 02 c3 f0
{150} 21 e8 18 80 02 c3 18 fb 9c fc 33 08 11 84 81 ff f0 b7 f8 [Temp 11.8 C  Hum 81%]
{149} 83 ae 18 80 02 c3 18 fc 78 fc 29 28 ff ff ff fe 03 98 00
{150} 5c e4 18 80 02 c3 18 fb ba fc 26 98 11 84 81 ff f0 16 00 [Temp 11.8 C  Hum 81%]
{148} d0 bd 18 80 02 c3 18 f9 ad fa 26 48 ff ff ff fe 02 ff f0
Wind and Temperature/Humidity or Rain:
    DIGEST:8h8h ID:8h8h8h8h FLAGS:4h BATT:1b CH:3d WSPEED:~8h~4h ~4h~8h WDIR:12h ?4h TEMP:8h.4h ?4h HUM:8h UV?~12h ?4h CHKSUM:8h
    DIGEST:8h8h ID:8h8h8h8h FLAGS:4h BATT:1b CH:3d WSPEED:~8h~4h ~4h~8h WDIR:12h ?4h RAINFLAG:8h RAIN:8h8h UV:8h8h CHKSUM:8h
Digest is LFSR-16 gen 0x8810 key 0x5412, excluding the add-checksum and trailer.
Checksum is 8-bit add (with carry) to 0xff.
Notes on different sensors:
- 1910 084d 18 : RebeckaJohansson, VENTUS W835
- 2030 088d 10 : mvdgrift, Wi-Fi Colour Weather Station with 5in1 Sensor, Art.No.: 7002580, ff 01 in the UV field is (obviously) invalid.
- 1970 0d57 18 : danrhjones, bresser 5-in-1 model 7002580, no UV
- 18b0 0301 18 : konserninjohtaja 6-in-1 outdoor sensor
- 18c0 0f10 18 : rege245 BRESSER-PC-Weather-station-with-6-in-1-outdoor-sensor
- 1880 02c3 18 : f4gqk 6-in-1
- 18b0 0887 18 : npkap

Parameters:

 msg     - Pointer to message
 msgSize - Size of message
 pOut    - Pointer to WeatherData

 Returns:

 DECODE_OK      - OK - WeatherData will contain the updated information
 DECODE_DIG_ERR - Digest Check Error
 DECODE_CHK_ERR - Checksum Error

*/
DecodeStatus decodeBresser6In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *pOut) {
    int const moisture_map[] = {0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99}; // scale is 20/3
    
    // LFSR-16 digest, generator 0x8810 init 0x5412
    int chkdgst = (msg[0] << 8) | msg[1];
    int digest  = lfsr_digest16(&msg[2], 15, 0x8810, 0x5412);
    if (chkdgst != digest) {
        //decoder_logf(decoder, 2, __func__, "Digest check failed %04x vs %04x", chkdgst, digest);
        Serial.print("Digest check failed - ");
        Serial.print(chkdgst, HEX);
        Serial.print(" vs ");
        Serial.println(digest, HEX);
        return DECODE_DIG_ERR;
    }
    // Checksum, add with carry
    int chksum = msg[17];
    int sum    = add_bytes(&msg[2], 16); // msg[2] to msg[17]
    if ((sum & 0xff) != 0xff) {
        //decoder_logf(decoder, 2, __func__, "Checksum failed %04x vs %04x", chksum, sum);
        Serial.print("Checksum failed - ");
        Serial.print(chksum, HEX);
        Serial.print(" vs ");
        Serial.println(sum, HEX);
        return DECODE_CHK_ERR;
    }

    pOut->sensor_id  = ((uint32_t)msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | (msg[5]);
    pOut->s_type     = (msg[6] >> 4); // 1: weather station, 2: indoor?, 4: soil probe
    pOut->battery_ok = (msg[6] >> 3) & 1;
    pOut->chan       = (msg[6] & 0x7);

    // temperature, humidity, shared with rain counter, only if valid BCD digits
    pOut->temp_ok  = msg[12] <= 0x99 && (msg[13] & 0xf0) <= 0x90;
    if (pOut->temp_ok) {
        int temp_raw   = (msg[12] >> 4) * 100 + (msg[12] & 0x0f) * 10 + (msg[13] >> 4);
        float temp_c   = temp_raw * 0.1f;
        if (temp_raw > 600)
            temp_c = (temp_raw - 1000) * 0.1f;
    
        pOut->temp_c   = temp_c;
        pOut->humidity = (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
    }
    // apparently ff0(1) if not available
    pOut->uv_ok  = msg[15] <= 0x99 && (msg[16] & 0xf0) <= 0x90;
    if (pOut->uv_ok) {
        int uv_raw = ((msg[15] & 0xf0) >> 4) * 100 + (msg[15] & 0x0f) * 10 + ((msg[16] & 0xf0) >> 4);
        pOut->uv   = uv_raw * 0.1f;
    }
    int flags  = (msg[16] & 0x0f); // looks like some flags, not sure

    //int unk_ok  = (msg[16] & 0xf0) == 0xf0;
    //int unk_raw = ((msg[15] & 0xf0) >> 4) * 10 + (msg[15] & 0x0f);

    // invert 3 bytes wind speeds
    msg[7] ^= 0xff;
    msg[8] ^= 0xff;
    msg[9] ^= 0xff;
    pOut->wind_ok = (msg[7] <= 0x99) && (msg[8] <= 0x99) && (msg[9] <= 0x99);
    if (pOut->wind_ok) {
        int gust_raw              = (msg[7] >> 4) * 100 + (msg[7] & 0x0f) * 10 + (msg[8] >> 4);
        pOut->wind_gust_meter_sec = gust_raw * 0.1f;
        int wavg_raw              = (msg[9] >> 4) * 100 + (msg[9] & 0x0f) * 10 + (msg[8] & 0x0f);
        pOut->wind_avg_meter_sec  = wavg_raw * 0.1f;
        pOut->wind_direction_deg  = (((msg[10] & 0xf0) >> 4) * 100 + (msg[10] & 0x0f) * 10 + ((msg[11] & 0xf0) >> 4)) * 1.0f;
    }
    
    // rain counter, inverted 3 bytes BCD, shared with temp/hum, only if valid digits
    msg[12] ^= 0xff;
    msg[13] ^= 0xff;
    msg[14] ^= 0xff;
    pOut->rain_ok   = msg[12] <= 0x99 && msg[13] <= 0x99 && msg[14] <= 0x99;
    if (pOut->rain_ok) {
        int rain_raw    = (msg[12] >> 4) * 100000 + (msg[12] & 0x0f) * 10000
                + (msg[13] >> 4) * 1000 + (msg[13] & 0x0f) * 100
                + (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
        pOut->rain_mm   = rain_raw * 0.1f;
    }
    pOut->moisture_ok = false;
    if (pOut->s_type == 4 && pOut->temp_ok && pOut->humidity >= 1 && pOut->humidity <= 16) {
        pOut->moisture_ok = true;
        if (pOut->moisture_ok) {
            pOut->moisture = moisture_map[pOut->humidity - 1];
        }
    }
    return DECODE_OK;
}

//
// Generate sample data for testing MQTT publishing -
// use with #define _DEBUG_MQTT_
//
bool genWeatherdata(WeatherData_t *pOut)
{
    pOut->sensor_id = 0xff;
    pOut->temp_c = 22.2f;
    pOut->humidity = 55;
    pOut->wind_direction_deg = 333;
    pOut->wind_gust_meter_sec = 44.4f;
    pOut->wind_avg_meter_sec = 11.1f;
    pOut->rain_mm = 9.9f;
    pOut->battery_ok = true;

    return true;
}


#ifdef _DEBUG_MODE_
void printRawdata(uint8_t *msg, uint8_t msgSize) {
    Serial.println("Raw Data:");
    for (uint8_t p = 0 ; p < msgSize ; p++) {
        Serial.printf("%02X ", msg[p]);
    }
    Serial.printf("\n");
}
#endif


//
// Get weather data from CC1101 receiver and decode it
//
bool getWeatherdata(void)
{
    uint8_t recvData[27];
    int state = radio.receive(recvData, 27);
    bool decode_ok = false;

    Serial.println(F("getWeatherdata()"));
    if (state == RADIOLIB_ERR_NONE) {
        // Verify last syncword is 1st byte of payload (see above)
        if (recvData[0] == 0xD4) {
            #ifdef _DEBUG_MODE_
                // print the data of the packet
                Serial.print("[CC1101] Data:\t\t");
                for(int i = 0 ; i < sizeof(recvData) ; i++) {
                    Serial.printf(" %02X", recvData[i]);
                }
                Serial.println();

                Serial.printf("[CC1101] R [0x%02X] RSSI: %f LQI: %d\n", recvData[0], radio.getRSSI(), radio.getLQI());
            #endif

            // FIXME: Keep previous values in case any data does not get updated
            //weatherData = { 0 };

            #ifdef _DEBUG_MODE_
                printRawdata(&recvData[1], sizeof(recvData));
            #endif
            
            // Decode the information - skip the last sync byte we used to check the data is OK
            #ifdef BRESSER_6_IN_1
                decode_ok = (decodeBresser6In1Payload(&recvData[1], sizeof(recvData) - 1, &weatherData) == DECODE_OK);
            #else
                decode_ok = (decodeBresser5In1Payload(&recvData[1], sizeof(recvData) - 1, &weatherData) == DECODE_OK);
          
                // Fixed set of data for 5-in-1 sensor
                weatherData.temp_ok     = true;
                weatherData.uv_ok       = false;
                weatherData.wind_ok     = true;
                weatherData.rain_ok     = true;
                weatherData.moisture_ok = false;
            #endif
          
            if (decode_ok) {
                const float METERS_SEC_TO_MPH = 2.237;
                Serial.printf("Id: [%8X] Battery: [%s] ",
                    weatherData.sensor_id,
                    weatherData.battery_ok ? "OK " : "Low");
                #ifdef BRESSER_6_IN_1
                    Serial.printf("Ch: [%d]", weatherData.chan);
                #endif
                if (weatherData.temp_ok) {
                    Serial.printf("Temp: [%3.1fC] Hum: [%3d%%] ",
                        weatherData.temp_c,
                        weatherData.humidity);
                } else {
                    Serial.printf("Temp: [---.-C] Hum: [---%] ");
                }
                if (weatherData.wind_ok) {
                    Serial.printf("Wind max: [%3.1fm/s] Wind avg: [%3.1fm/s] Wind dir: [%4.1fdeg] ",
                         weatherData.wind_gust_meter_sec,
                         weatherData.wind_avg_meter_sec,
                         weatherData.wind_direction_deg);
                } else {
                    Serial.printf("Wind max: [--.-m/s] Wind avg: [--.-m/s] ");
                }
                if (weatherData.rain_ok) {
                    Serial.printf("Rain: [%6.1fmm] ",  
                        weatherData.rain_mm);
                } else {
                    Serial.printf("Rain: [-----.-mm] "); 
                }
                if (weatherData.moisture_ok) {
                    Serial.printf("Moisture: [%2d%%]",
                        weatherData.moisture);
                }
                Serial.printf("\n");
                //printf("{\"sensor_type\": \"bresser-5-in-1\", \"sensor_id\": %d, \"battery\": \"%s\", \"temp_c\": %.1f, \"hum_pc\": %d, \"wind_gust_ms\": %.1f, \"wind_speed_ms\": %.1f, \"wind_dir\": %.1f, \"rain_mm\": %.1f}\n",
                //       sensor_id, !battery_low ? "OK" : "Low",
                //       temperature, humidity, wind_gust, wind_avg, wind_direction_deg, rain);
            } // if (decode_ok)
            else {
                #ifdef _DEBUG_MODE_
                    Serial.printf("[CC1101] R [0x%02X] RSSI: %f LQI: %d\n", recvData[0], radio.getRSSI(), radio.getLQI());
                #endif
            }
        } // if (recvData[0] == 0xD4)
        else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            #ifdef _DEBUG_MODE_
                Serial.print("T");
            #endif
        } // if (state == RADIOLIB_ERR_RX_TIMEOUT)
        else {
            // some other error occurred
            Serial.printf("[CC1101] Receive failed - failed, code %d\n", state);
        }
    } // if (state == RADIOLIB_ERR_NONE)

    return decode_ok;
}


void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    Serial.println(F("[CC1101] Initializing ... "));
    int state = radio.begin(868.35, 8.22, 57.136417, 270.0, 10, 32);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("success!");
        state = radio.setCrcFiltering(false);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[CC1101] Error disabling crc filtering: [%d]\n", state);
            while (true)
                ;
        }
        state = radio.fixedPacketLengthMode(27);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[CC1101] Error setting fixed packet length: [%d]\n", state);
            while (true)
                ;
        }
        // Preamble: AA AA AA AA AA
        // Sync is: 2D D4 
        // Preamble 40 bits but the CC1101 doesn't allow us to set that
        // so we use a preamble of 32 bits and then use the sync as AA 2D
        // which then uses the last byte of the preamble - we recieve the last sync byte
        // as the 1st byte of the payload.
        state = radio.setSyncWord(0xAA, 0x2D, 0, false);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[CC1101] Error setting sync words: [%d]\n", state);
            while (true)
                ;
        }
    } else {
        Serial.printf("[CC1101] Error initialising: [%d]\n", state);
        while (true)
            ;
    }

    // Configure status output pins
    pinMode(PIN_SENSOR_DECOK, OUTPUT);
    pinMode(PIN_TTN_TXCOMPL, OUTPUT);
    
    weatherData = { 0 };
    Serial.println(F("[CC1101] Setup complete - awaiting incoming messages..."));

    // LMIC init
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    #ifdef CC1101_DEBUG
        getWeatherdata();
    #else
        os_runloop_once();
    #endif
}
