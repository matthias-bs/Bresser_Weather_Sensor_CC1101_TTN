# Bresser_Weather_Sensor_CC1101_TTN
Bresser 5-in-1/6-in-1 868 MHz Weather Sensor Radio Receiver based on CC1101, ESP8266/ESP32 and RFM95W - provides data via LoRaWAN to The Things Network

Based on:
- [Bresser5in1-CC1101](https://github.com/seaniefs/Bresser5in1-CC1101) by [Sean Siford](https://github.com/seaniefs)
- [RadioLib](https://github.com/jgromes/RadioLib) by [Jan Gromeš](https://github.com/jgromes)
- [MCCI LoRaWAN LMIC library](https://github.com/mcci-catena/arduino-lmic) by Thomas Telkamp and Matthijs Kooijman / Terry Moore, MCCI
- [Lora-Serialization](https://github.com/thesolarnomad/lora-serialization) by [Joscha Feth](https://github.com/joscha)

## Weather Stations

* [BRESSER Weather Center 5-in-1](https://www.bresser.de/en/Weather-Time/Weather-Center/BRESSER-Weather-Center-5-in-1-black.html)
* [BRESSER Professional WIFI colour Weather Center 5-in-1 V](https://www.bresser.de/en/Weather-Time/WLAN-Weather-Stations-Centers/BRESSER-Professional-WIFI-colour-Weather-Center-5-in-1-V.html)

The Bresser 5-in-1 Weather Stations seem to use two different protocols. Select the appropriate decoder by (un-)commenting `#define BRESSER_6_IN_1` in the source code.

| Model         | Decoder Function                |
| ------------- | ------------------------------- |
| 7002510..12   | decodeBresser**5In1**Payload()  |
| 7002585       | decodeBresser**6In1**Payload()  |

## Hardware 

### ESP32


### CC1101

[Texas Instruments CC1101 Product Page](https://www.ti.com/product/CC1101)

**Note: CC1101 Module Connector Pitch is 2.0mm!!!**

Unlike most modules/breakout boards, most (if not all) CC1101 modules sold on common e-commerce platforms have a pitch (distance between pins) of 2.0mm. To connect it to breadboards or jumper wires with 2.54mm/100mil pitch (standard), the following options exist:

* solder wires directly to the module
* use a 2.0mm pin header and make/buy jumper wires with 2.54mm at one end and 2.0mm at the other (e.g. [Adafruit Female-Female 2.54 to 2.0mm Jumper Wires](https://www.adafruit.com/product/1919))
* use a [2.0mm to 2.54 adapter PCB](https://www.amazon.de/Lazmin-1-27MM-2-54MM-Adapter-Platten-Brett-drahtlose-default/dp/B07V873N52)

**Note 2: Make sure to use the 868MHz version!**


### Adafruit RFM95W LoRa Radio Transceiver Breakout

The LoRa transceiver and antenna must be selected according to the [Frequency Plans by Country](https://www.thethingsnetwork.org/docs/lorawan/frequencies-by-country/).

* [ADA3072](https://www.adafruit.com/product/3072) - 868/915 MHz version
* [ADA3073](https://www.adafruit.com/product/3073) - 433 MHz version
* RF connector
* Antenna


## Software

Software for Arduino ESP32/ESP8266: [Bresser_WeatherStation_CC1101_TTN.ino](Bresser_WeatherStation_CC1101_TTN.ino)

The payload data structure is encoded into bytes using a [modified version of Lora-Serialization](src/LoRa_Serialization/src).

**Note: `writeRawFloat()` is missing in the default Arduino library version!**

**Note2: `writeUint32()` has been added to the customized version.**

### The Things Network Decoder

Decode payload (a sequence of bytes) into data structures which are readable/suitable for further processing:
paste [ttn_decoder.js](ttn_decoder.js)  as "Custom Javascript formatter" to "Payload Formatters" -> "Uplink" on The Things Network Console.

### TTN-MQTT-Integration

How to receive and decode the payload with an MQTT client -
see https://www.thethingsnetwork.org/forum/t/some-clarity-on-mqtt-topics/44226/2

V3 topic:

`v3/<ttn app id><at symbol>ttn/devices/<ttn device id>/up`

  
v3 message key field jsonpaths:
  
```
<ttn device id> = .end_device_ids.device_id
<ttn app id> = .end_device_ids.application_ids.application_id  // (not including the <at symbol>ttn in the topic)
<payload> = .uplink_message.frm_payload
```  


JSON-Path with Uplink-Decoder (see [ttn_decoder.js](ttn_decoder.js))

`.uplink_message.decoded_payload.bytes.<variable>`
