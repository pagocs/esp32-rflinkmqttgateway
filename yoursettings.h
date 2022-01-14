#ifndef _YOURSETTINGS_H
#define _YOURSETTINGS_H

//------------------------------------------------------------------------------
// You should set your usernames and passwords here

#define MY_WIFI_AP "your ap name"
#define MY_WIFI_PSW "password"

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

#define MQTT_USER "user"
#define MQTT_PSW "password"

#define VERSION "Ver: 1.1.7"

// This define is enable the multi gateway mode
// #define MULTIGATEWAY

// Temporary solutions (since 3 years :)
// You can put here the devices what you want to filter out

const char * deviceignorelist[] = {

    // Example:
    // "SelectPlus3" , "Alecto V2" , "TriState" ,
    // "Warema" , "Drayton", "Keeloq" ,
    // "Auriol V2",

     NULL
};

// If you do not define the server the function isaautomatically turned off
// Syslog server connection info

// #define SYSLOG_SERVER "192.168.0.100"
// #define SYSLOG_PORT 514

// If it is defined, the MQTT send and recive will blink the specified
// pin to provide a visual feedback for the communication

// #define MQTTLEDDEBUG

// You can define the PIN where the LED is conected. If you not defined the default
// is 12 (but check in the MQTT.ino file please..)

// #define MQTTLEDPIN xx

// FW R49: Accurite899 mm to tickcount conversion for legacy reason
// #define ACURITE899CONVERSION

#endif // _YOURSETTINGS_H
