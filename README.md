> :warning: This repository is deprecated and no longer maintained. 
> The recommended alternative is [BresserWeatherSensorTTN](https://github.com/matthias-bs/BresserWeatherSensorTTN)
> which provides a much cleaner design (separation between Weather Sensor and TTN code) and allows 
> to share a RFM95W/SX1276 radio transceiver between weather data reception (FSK mode) and TTN communication (LoRaWAN mode).

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

Wiring:

| ESP32        | Misc         | CC1101 |       | RFM95W         |
| ------------ | ------------ | -------|------ | -------------- |
| Name         |              | Pin#   | Func. |                |  
| 3V3          |              | 1      | Vcc   | VIN            |
| GND          |              | 2      | GND   | GND            |
| GPIO23/MOSI2 |              | 3      | MOSI  | MOSI           |
| GPIO18/SCK2  |              | 4      | SCK   | SCK            |
| GPIO19/MISO2 |              | 5      | MISO  | MISO           |
| GPIO04       |              | 6      | GDO2  |                |
| GPIO26       |              | 7      | GDO0  |                |
| GPIO15       |              | 8      | CS    |                |
| GPIO27       |              |        |       | CS             |
| GPIO21       |              |        |       | G0             |
| GPIO33       |              |        |       | G1             |
| GPIO25       |              |        |       | G2             |
| GPIO32       |              |        |       | RST            |
| GPIO12       | SENSOR_DECOK[¹] |        |       |                |
| GPIO13       | TTN_TXCOMP[²] |        |       |                |

[¹]: SENSOR_DECOK: Weather Sensor Message Decoding o.k.

[²]: TTN_RXCOMP: The Things Network Transmisson Completed

You might want to connect an LED (with a resistor to GND) to those pins. 

A buffer capacitor of ~10µF across the supply pins of the RFM95W module is recomended.


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

**Note:**
In a 868MHz LoRaWAN region, the RFM95W ~~could probably~~ can be used to receive the weather sensor data. Thus, the transceiver module would be switched between FSK mode for sensor data reception and LoRaWAN mode for the uplink to The Things Network.

### Antennas and RF Connectors

The required antenna depends on the signal path between weather sensor and CC1101 receiver or RFM95W transceiver and LoRaWAN gateway, respectively. Some options are:
* wire antenna
* spring antenna (helical wire coil)
* rubber antenna

See [Adafruit Tutorial - Antenna Options](https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/antenna-options) for wire antenna lengths and uFL connector soldering.

The [Data Alliance](https://www.data-alliance.net/mhf-series-mhf1-mhf2-mhf3-mhf4/) website helped to sort out my RF connector confusion:

> Applications of MHF Connectors & Cables
>
> The MHF series of RF micro-connectors (mating heights listed below are the maximum):
> * MHF1 (also known as MHF) has a Mating Height of 2.5mm
> * MHF2 has a Mating Height of 2.1mm
> * MHF3 has a Mating Height of 1.6mm
> * MHF4 has a Mating Height of 1.2mm
>
> MHF3 connector is compatible with a W.FL connector while MHF2 connector is equivalent of U.FL connector. The MHF4 cable connector is the smallest while MHF1 connector is the largest which is comparable to a U.FL connector.

Personally I prefer the SMA connector over the uFL connector -  but be aware of the (usual) male/female connector types and the normal/reverse polarity types. See [SMA vs RP-SMA what is the difference?](https://forum.digikey.com/t/sma-vs-rp-sma-what-is-the-difference/550) by Digikey.

**Note: Depending on your region, both the CC1101 weather sensor receiver and the RFM95W Lora Transceiver might use the same frequency. In this case, please keep some distance (min. ~20cm) between the antennas to avoid overloading of the receiver.**

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
