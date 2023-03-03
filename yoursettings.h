#ifndef _YOURSETTINGS_H
#define _YOURSETTINGS_H

/*
 JSON
 https://techtutorialsx.com/2017/04/26/esp32-parsing-json/
 https://github.com/bblanchon/ArduinoJson
 Examples: https://arduinojson.org/example/?utm_source=github&utm_medium=readme

 LOGGING:
    http://esp-idf.readthedocs.io/en/latest/api-reference/system/log.html
    https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/log/esp_log.h

OTA Update
/Users/pagocs/Documents/Arduino/hardware/espressif/esp32/tools/espota.py -d -i 192.168.1.40 -p 3232 -f /Users/pagocs/Documents/Arduino/projects/rflinkgateway/rflinkgateway.ino.lolin32.bin
/Users/pagocs/Documents/Arduino/hardware/espressif/esp32/tools/espota.py -d -i 192.168.1.42 -p 3232 -f /Users/pagocs/Documents/Arduino/projects/rflinkgateway/rflinkgateway.ino.lolin32.bin

*/

//------------------------------------------------------------------------------
// You should set your usernames and passwords here
//

#define MY_WIFI_AP "Hades"
#define MY_WIFI_PSW "Zombi33265@giliszta"

// You can define the MQTT broker address and port or you can use the
// automatic discovery.

// #define MQTT_BROKER 192.168.0.100
// #define MQTT_PORT 1883

// For the automatic discovery you need to setup a mosquitto broker with
// discoverable over mDNS.
// Resources
// You can find the AVAHI service config file: mosquitto.service in the poroject folder
//
// AVAHI: http://dagrende.blogspot.com/2017/02/find-mqtt-broker-without-hard-coded-ip.html
// Mosquitto:
// https://www.domoticz.com/wiki/MQTT
// in docker: https://hub.docker.com/_/eclipse-mosquitto
// Raspberry: https://randomnerdtutorials.com/how-to-install-mosquitto-broker-on-raspberry-pi/


#define MQTT_USER "admin"
#define MQTT_PSW "admin1234"

#define VERSION "Ver: 1.1.8"

// This define is enable the multi gateway mode
#define MULTIGATEWAY

// !!!!!!!!!!!!!!! Temporary solutions (since 3 years :) !!!!!!!!!!!!!!!
// You can put here the devices what you want to filter out

const char * deviceignorelist[] = {
     "SelectPlus3" , "Alecto V2" , "TriState" ,
     "Doorbell" , "Bosch" , "Eurodomest" ,
     "FineOffset", "EV1527" , "Aster" ,
     "Mertik_GV60" , "SelectPlus" , "Avidsen" ,
     "TRC02RGB", // RFLink Gateway	00FFFFFF	255	TRC02RGB	Color Switch	RGBW
     "Warema" , "Drayton", "Keeloq" ,
     "Mebus" , "Xiron" , "Tunex" ,
     "V2Phoenix" , "Conrad RSL" , "LaCrosse TX3",
     "Auriol V2", "Renkforce E_TA", "Unknown",
     "FA500" , "Deltronic"

     NULL
};
// If you do not define the server the function isaautomatically turned off
// Syslog server connection info

#define SYSLOG_SERVER "iris.pagocs.com"
#define SYSLOG_PORT 514

// If it is defined, the MQTT send and recive will blink the specified
// pin to provide a visual feedback for the communication
#define MQTTLEDDEBUG
// You can define the PIN where the LED is conected. If you not defined the default
// is 12 (but check in the MQTT.ino file please..)
// #define MQTTLEDPIN xx

// FW R49: Accurite899 mm to tickcount conversion for legacy reason
// #define ACURITE899CONVERSION

#endif // _YOURSETTINGS_H
