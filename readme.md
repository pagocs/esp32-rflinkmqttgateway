# Notice
For use this gateway with Domoticz it should contains the RFLINK MQTT gateway device
WHICH IS NOT YET A PART OF the official Domoticz version but it is under progress,
that was the reason why this project is raised to public anyway... :)

# Summary
This project is implement an MQTT based network interface for the great
RFLINK Gateway <https://www.rflink.nl> to ensure an independent installation
and seamless integration over wifi with a Domoticz <https://www.domoticz.com> home
automation system. On the other hand thanks for the MQTT broker you can use the
data or control the RFLINK devices with multiple Domoticz instances or any other
MQTT capable methods.

The MQTT RFLink gateway is based on an cheap ESP32 mcu (WROOM-32) what you should connect
to RFLINK Gateway (<https://www.nodo-shop.nl/en/rflink-/127-rflink-arduino-antenna.html>)
serial lines as you can see the diagram and the photos bellow.

<img src="https://github.com/pagocs/esp32-rflinkmqttgateway/blob/main/assets/images/wiring_diagram.jpg" width="300" height="560">

<img src="https://github.com/pagocs/esp32-rflinkmqttgateway/blob/main/assets/images/IMG_5388.jpeg" alt="Example setup" width="300" height="486">

The communication between the Arduino and the ESP32 is goes trough the Arduino
Tx and Rx line. I connect this line directly to each other because ESP32 lines
are 5 volt tolerant as I known.

My setup is working more than 2 years now (2022)
without any problem but if you would like follow the official guidance here is  a
good starting point: <https://create.arduino.cc/projecthub/nidhi17agarwal/uart-communication-between-arduino-uno-and-esp32-1170d5>.

The RFLINK 433 v 1.1.4 board have the Tx and Rx line connection (as you can check
on my pictures) but on the earlier boards may missing this pins, so check this carefully.

The MQTT communication is based on the standard RFLINK protocol what you can find here:
<https://www.rflink.nl/protref.php> just packed those commands to MQTT packets.

The IN/OUT topics are fixed and it is defined as:
* RFLINK_RECEIVE_TOPIC    "rflink/in"
* RFLINK_CONTROL_TOPIC    "rflink/out"

## Logging
The ESP32 firmware have a moderate logging capability what you can access over the
network using linux nc command (for example: nc 192.168.0.42 80) or a browser with
using the board ip address.

## MQTT broker
Yo can use the gateway with static broker what you should define in compile time. An another way the automatic discover. For this you need to setup a mosquitto broker with discoverable over mDNS.

On linux based systems you should setup:
* The mosquitto broker
* and setup AVAHI which provide the mDNS service. You should install the AVAHI service and copy the mosquitto.service file to the /etc/avahi/services folder (on the system where the mosquitto and AVAHI is running) and restart the avahi service (for example:
    sudo systemctl restart avahi-daemon).

## Flashing the firmware
Currently pre compiled firmware is not available because changing network settings is not implemented just with text editor... :)

So for the firmware installation you should compile for yourself but just first time. If you are flashed the ESP32 successfully you can use the OTA for the subsequent updates.

Ota update example:
'''
espota.py -d -i 192.168.0.42 -p 3232 -f ~/Documents/Arduino/projects/rflinkgateway/rflinkgateway.ino.lolin32.bin
'''

# Compile and dependencies
The firmware itself is compatible with Arduino IDE and have the following
dependencies:

* Arduino ESP32 support <https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html>
* Common modules        <https://github.com/pagocs/esp32-common>
                          You should copy this modules into a folder in your arduino library folder
* ArduinoJson 5.13.5    <https://arduinojson.org>
* PubSubClient 2.8.0    <https://github.com/knolleary/pubsubclient>
* Standard Arduino/ESP32 modules

## Before you can compile

* Connect the serial USB converter to your machine and the ESP32 board
* Select the USB port
* Setup the board type to WEMOS Lolin32 and partiton scheme could be default.
* Setup the upload speed based on your experience but you can use 921600 bravely
* Setup the connection parameters in the rflinkgateway.ino source file
    * WIFI
        * MY_WIFI_AP "ap"
        * MY_WIFI_PSW "password"
    * MQTT broker
        * (optional) If you are defined the server the MQTT will work with the defined server, otherwise you should setup a mosquitto broker with mDNS discoverable. See more information above.
            * MQTT_BROKER 192.168.0.142
            * MQTT_PORT 1883
        * MQTT_USER "user"
        * MQTT_PSW "password"

Optionally you can define device type names which will be filtered out and do not
flood your device database in Domoticz system.

# How to use it
For use this gateway with Domoticz you should setup a "RFLink Gateway MQTT" device on your hardware tab.

## Parameters
* Remote server: it contains the MQTT broker IP address (or adresses if you heve "backup" broker, in tthis case you should separate the addresses with semivolon look like this: 192.168.0.111;192.168.0.222)
* Port: MQTT broker port, default is 1883
* Username and password: he credential for MQTT broker
* CA Filename and TLS version: parameters for secure connection, but this was not tested by me yet.

<img src="https://github.com/pagocs/esp32-rflinkmqttgateway/blob/main/assets/images/domoticz_settings.jpg" width="300" height="560">
