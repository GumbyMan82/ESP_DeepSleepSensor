# ESP_DeepSleepSensor
Code for an ESP32 based sensor using any kind of analog sensor.
ESP is going to deep sleep between the sensor measurements.
The measured data are transmitted to a MQTT broker and with MQTT the sleep interval can also be configured.
The maximum sleep time is set to 3600 seconds, the default sleep time is configured to 300s.
So if the broker sets invalid data (such as values that cannot be casted or invalid values), the default sleep time is applied.
The sleep time is stored persistent in memory (NVS)
## Required hardware / software
- ESP32.
- Some kinde of sensor
- MQTT broker

Analog input of the sensor is currently hard coded to analog input 0
Digital output for triggering the sensor is currently hard coded to output 26

### Required libraries
```C++
#include "PubSubClient.h" // tested with version 2.7
#include "ESPAsyncWebServer.h" //tested with version 1.2.3
```

## MQTT message format publish
The  topic can be selected freely in the configuration interface
The topic for publishing will be extended by "/State".
e.g. if you configure ```Home/MyGardenSensor``` the state message will be posted to ```Home/MyGardenSensor/State```

The message payload is a JSON formatted string
```JSON
{
    "state" : "ON|OFF|SLEEP", 
    "value" : "%d", 
    "SleepTime" : "%d"
}
```
```state``` state of the ESP. ON --> running, OFF --> when performing a soft reset or within last will, SLEEP --> when in deep sleep mode
```value``` is the raw sensor value. Calibration (e.g. what is moist, what is dry) needs to be done on the other system that processes the sensor readings
```SleepTime``` is the amount of seconds that the ESP is currently going to sleep. 


## Changing the sleep tme
The MQTT client automatically subscribes to ```<configured_topic>/SleepTime``` for changing the time in seconds for which the ESP should go into deep sleep. Maximum value is 3600s. Any value outside of 0...3600s will fallback to 
```C++
/**
 * The default sleep time in [s]
 */ 
#define DEF_TIME_TO_SLEEP  5*60 
```
## Change operation mode

The MQTT client automatically subscribes to ```<configured_topic>/Mode```.
The following modes are possible
Command|Description
---|---
AP | ESP starts as an access point with the IP 192.168.1.4 and starts the webserver for configuration
CONFIG| ESP starts only the webserver for configuration

Since during normal operation the ESP is in deep sleep quite often, those messages must be retained. After recieving an message, an empty payload is automatically sent to the topic. Otherwise the broker would still send the ESP into CONFIG or AP mode until something else changes the payload of the retained message.


## Webserver
The configuration page of the webserver is being read from SPIFFS. Therefore the data folder from this repository must be uploaded to the ESPs file system.
The Webserver listens to __port 80__.
After applying the configuration in the webserver, the ESP is restarted

## Acces point mode
In AP mode, the ESP creates a WLAN with the SSID ```ESP32-DeepSleepSensor```. No credentials are required.
The IP of the ESP is ```192.168.4.1```

## Statemachine
STATE | CONDITION | NEXT STATE | DESCRIPTION
--- | --- | --- | ---
```START```|_(none)_ | ```READ_CONFIG``` | Boot state 
 ```READ_CONFIG``` | configuration OK | ```CONNECT_WIFI``` | If configuration parameter are OK, try to connect to WiFi
 ```READ_CONFIG```| WiFi parameter missing | ```START_AP``` | Starts the ESP in access point mode
 ```READ_CONFIG```|MQTT parameter missing | ```START_WEBSERVER``` | Start webserver for configuration
 ```CONNECT_WIFI```|connection successful | ```CONNECT_MQTT``` | Try to connect to MQTT if WiFi connection was successful
 ```CONNECT_WIFI``` |connection failed | ```START_AP``` | Starts the ESP in access point mode if no WiFi connection could be established
 ```CONNECT_MQTT``` | connection succesful | ```OPERATIONAL``` | After MQTT connection switch to normal operation
 ```CONNECT_MQTT```|connection failed | ```START_WEBSERVER``` | Start webserver for configuration
 ```START_AP``` | _(none)_ |```START_WEBSERVER```|after starting in AP mode, the webserver is started for configuration
 ```START_WEBSERVER``` | _(none)_ | ```WEBSERVER_RUNNING``` | Webserver is running
 ```OPERATION``` | _(none)_ | ```DEEP_SLEEP``` | A MQTT message with the status and the sensor readings is published in this state. Afterwards, ESP is being put into deep sleep
 ```DEEP_SLEEP``` | _(none)_ | _(none)_ | sleeping. A state message is published before ESP goes to sleep

If the configuration is OK and connection to WiFi and MQTT is succesful, a normal state transition looks like the following
```START``` &rarr; ```READ_CONFIG``` &rarr; ```CONNECT_WIFI``` &rarr; ```CONNECT_WIFI``` &rarr; ```OPERATION``` &rarr; ```DEEP_SLEEP```
