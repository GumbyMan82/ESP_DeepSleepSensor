#include <Arduino.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "Preferences.h"
#include "DNSServer.h"
#include <string>
#include "ESPAsyncWebServer.h"
#include <SPIFFS.h>
#include <Ticker.h>

const char* version = "1.1";

//################# DEFINES ################
/**
 * conversion factor from microseconds in seconds
 * Noted in 64bit since the sleep function requires 64bit
 */ 
#define uS_TO_S_FACTOR 1000000LL


/** 
 * Maximum runtime after which the ESP reboots
 */
#define MAX_RUNTIME_S 1800


/**
 * Time at which the MQTT connection is being checked in WEBSERVER mode
 */
#define WEBSERVER_MQTT_RECHECK_TIME_S 60*5


/**
 * The default sleep time in [s]
 */ 
#define DEF_TIME_TO_SLEEP  5*60        /* Time ESP32 will go to sleep (in seconds) */

/**
 * Port at which a LED is connected. Used in DebugBlink()
 */ 
#define BLINK_PORT	2

/**
 * Port at which the MQTT broker runs
 */ 
#define MQTT_PORT		1883

/**
 * MQTT client ID
 */ 
#define MQTT_CLIENT_ID	"ESP32_1"

/**
 * State topic  --> TODO: See  Github Issue #1
 */ 
#define MQTT_STATE_TOPIC		"State"

/**
 * Topic to control the operation mode
 */ 
#define MQTT_MODE_TOPIC			"Mode"

/**
 * Amount of MQTT connection retries before the state machine switches to Configuration interface
 */ 
#define MQTT_CONNECTION_RETRIES	10

/**
 * Usually the MQTT client runs in his loop dwithin the loop function
 * Since we are going to deep_sleep we need to make sure that at least some loops are processed
 * in order to retrieve data from the broker. Maybe fewer runs are possible, but 10 does not hurt
 */ 
#define MAX_MQTT_LOOPS	10

/**
 * Each MQTT client loop has a delay in order to not run through the loops too fast and miss
 * data from the broker
 */ 
#define MQTT_LOOP_DELAY_MS 20

/**
 * Maximum deep sleep time in [s] = 1 hour
 */ 
#define MAX_SLEEP_TIME 3600

/**
 * Maximum number of connection retries before wifi connection fails and ESP goes into AP mode
 */ 
#define MAX_WIFI_CONNECT_RETRIES 5

/**
 * Define cycles in which the program will wait for WIFI connections. If that time is elapsed, WIFI is switched off and on again
 */ 
#define WIFI_CONNECT_WAIT_CYCLES 20

/**
 * MQTT State --> SLEEP
 */ 
#define STATE_SLEEP	(char*)"SLEEP"

/**
 * MQTT State --> ON
 */ 
#define STATE_ON	(char*)"ON"

/**
 * MQTT State --> OFF
 */ 
#define STATE_OFF	(char*)"OFF"
//################# DEFINES ################




//################# runtime data ################

/**
 * boot count after waking up
 */ 
RTC_DATA_ATTR int bootCount = 0;

/**
 * buffer for the sleep time read from preferences
 */ 
int SleepTimeSeconds = 0;

/**
 * Preferences --> persistent data
 */ 
Preferences preferences;

/**
 * the wifi client used in the entire project
 */ 
WiFiClient espClient;

/**
 * MQTT client using PubSubClient library
 */ 
PubSubClient MQTTClient(espClient);

/**
 * SSID of the wireless lan to connect to
 */ 
String WiFi_SSID = "";

/**
 * key for the wireless lan
 */ 
String WiFi_KEY = "";

/**
 * IP/Hostename of the MQTT broker
 */ 
String MqttBroker =	"";

/**
 * MQTT username
 */ 
String MqttUser ="";

/**
 * MQTT Password
 */ 
String MqttPassword =	"";

/**
 * MQTT Topic
 */
String MqttTopic = "";

/**
 * Analog input to which the sensor is connected to
 */ 
int AnalogInput = -1;

/**
 * Triggering output for the sensor
 */ 
int TriggerOutput = -1;

/**
 * State of the configuration data
 */ 
enum ConfigState
{
	CONFIG_OK = 0,
	WIFI_MISSING = 1,
	MQTT_MISSING = 2,
};

/**
 * Enum for the statemachine
 */ 
enum State
{
	START = 0,
	READ_CONFIG = 1,
	CONNECT_WIFI = 2,
	CONNECT_MQTT = 4,
	OPERATIONAL = 8,
	DEEP_SLEEP = 16,
	START_AP = 32, 
	START_WEBSERVER = 64,
	WEBSERVER_RUNNING = 128,
	NONE = 265,
	ERROR = 512,
	READ_SENSOR = 1024
};

/**
 * The current state of the statemachine
 */ 
State currentState;

/**
 * The next state of the statemachine
 */
State nextState; 

/**
 * shows if there was an error eading the config
 */
bool ConfigOK = false;

/**
 * shows if thw WIFI connection was successful
 */ 
bool WiFiConnectionOK = false;

/**
 * shows if the MQTT connection was successful
 */ 
bool MQTTConnectionOK = false;

/**
 * shows if there was a request to change the configuration
 */
bool RequestNewConfig = false;

/**
 * shows if there was a request to start the access point
 */
bool RequestStartAP = false; 

/**
 * Shows if the webserver was forced to start (--> by MQTT data)
 */ 
bool ForceStartWebserver = false;

/**
 * Flag for the Ticker 
 */ 
bool TickerElapsed = false;

/**
 * Flag for the MQTT connection check ticker
 * if this flag is true, the main loop should recheck the MQTT connection and
 * if succeeded, reboot the esp
 */ 
bool MQTTConnectionCheck = false;

/**
 * Flag for checking if the ESP runs in AP mode
 */ 
bool APMode = false;

/**
 * http server on port 80
 */ 
AsyncWebServer server(80);


/**
 * If set to true, the ESP will constantly read sensor data instead of going to deep sleep and reboot
 */ 
bool CyclicSensorReading = false;

/**
 * DNSServer for captive portal
 */
DNSServer dnsServer;

/**
 * Ticker that reboots the ESP when elapsed
 */
Ticker rebootTicker;


/**
 * Ticker that is used for rechecking the MQTT connection
 */ 
Ticker MQTTRecheckTicker;


/**
 * Analog value that is being read from the sensor
 */
uint16_t AnalogValue = 0;

/**
 * RSSI of the WiFI Signal
 */ 
int8_t RSSI = 0;

//################# runtime data ################




//################# const data ################
const char* AccesPoint_SSID = "ESP32-DeepSleepSensor";
const IPAddress apIP(192,168,1,1);
const IPAddress apSubnet(255,255,255,0);
const byte DNSPort = 53;
const char* PARAM_MESSAGE = "message";
//################# const data ################


//################# clases ################
class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>");
    response->print("<p>This is out captive portal front paaaage.</p>");
    response->printf("<p>You were trying to reach: http://%s%s</p>", request->host().c_str(), request->url().c_str());
    response->printf("<p>Try opening <a href='http://%s'>this link</a> instead</p>", WiFi.softAPIP().toString().c_str());
	response->print("<p>Or try to open SPIFFS <a href='http://index.html'>index.html</a></p>");
    response->print("</body></html>");
    //request->send(response);
	request->send(SPIFFS, "/index.html", "text/html");
  }
};
//################# clases ################


/**
 * Saves the sleep delay time to the preferences
 * @param delay The sleep delay in [s]
 */
void SaveSleepDelaySeconds(int delayTime){
	if(delayTime > 0 && delayTime <= MAX_SLEEP_TIME){
		preferences.putInt("SleepTime", delayTime);
	}
}


/**
 * returns the configured sleep time from preferences or DEF_TIME_TO_SLEEP
 * @return The sleep time in [s] or DEF_TIME_TO_SLEEP
 */
int GetSleepDelaySecondsOrDefault(){
	int i = preferences.getInt("SleepTime", DEF_TIME_TO_SLEEP);
	if(i > 0 && i <= MAX_SLEEP_TIME){
		return i;
	}
	else{
		return DEF_TIME_TO_SLEEP;
	}
}


/**
 * 	Debug function that blinks at the LED PORT
 */
void DebugBlink(int Repetitions){

	#ifdef DEBUG
	pinMode(BLINK_PORT, OUTPUT);
	for(int i = 0; i<Repetitions; i++){
		digitalWrite(BLINK_PORT, 1);
		delay(200);
		digitalWrite(BLINK_PORT, 0);
		delay(200);
	}
	delay(500);
	#endif
}


/**
 * Method to print the reason by which ESP32
 * has been awaken from sleep
 */
void print_wakeup_reason(){
	esp_sleep_wakeup_cause_t wakeup_reason;

	wakeup_reason = esp_sleep_get_wakeup_cause();

	switch(wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
	case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
	case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
	case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
	default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
	}

	Serial.println();
	Serial.println("--------------------");
}


/**
 * Reads the analog value of the sensor. Switches the sensor on using a digital output
 * 
 * @param ADCChannel Analog channel to which the sensor is connected
 * @param PowerChannel Digital output that powers the sensor
 */
uint16_t ReadAnalogValue(uint8_t ADCChannel, int PowerChannel){
	
	//fail fast if parameters are wrong
	if(ADCChannel < 0 || ADCChannel > 255){
		Serial.println("Invalid ADC Channel - exiting");
		Serial.println(ADCChannel);
		return 0;
	}
		
	
	if(PowerChannel < 0 || PowerChannel > 255){
		Serial.println("Invalid trigger channel - exiting");
		Serial.println(PowerChannel);
		return 0;
	}
		
	
	int i = 0;
	int filter = 10;
	uint32_t tmp = 0;
	float f_value = 0.0;
	pinMode(TriggerOutput, OUTPUT);
	
	digitalWrite(PowerChannel, HIGH);
	delay(100);
	for(i = 0; i<filter; i++)
	{
		tmp += analogRead(ADCChannel);
	}

	f_value = (tmp*1.0)/filter;
	
	digitalWrite(PowerChannel, LOW);

	return (uint16_t)f_value;
}


char* DataToJSonString(char* state, uint16_t analogValue, int SleepDelay){
	//static needed since its a volatile memory otherwise
	static char cJSONString[80];
	cJSONString[0] = '\0';
	sprintf(cJSONString, "{\"state\" : \"%s\", \"value\" : \"%d\", \"SleepTime\" : \"%d\", \"SignalQuality\" : \"%d\"} ",state ,AnalogValue, GetSleepDelaySecondsOrDefault(), RSSI);
	Serial.println(cJSONString);
	return cJSONString;
}


/**
 * Publishes the data via MQTT in JSON format
 */ 
void PublishState(char* state){
	if(MQTTClient.connected())
	{
		//char cJSONString[80];
		//cJSONString[0] = '\0';
		//sprintf(cJSONString, "{\"state\" : \"%s\", \"value\" : \"%d\", \"SleepTime\" : \"%d\"} ",state ,AnalogValue, GetSleepDelaySecondsOrDefault());
		//Serial.println(cJSONString);
		//MQTTClient.publish(String(MqttTopic + "/" + MQTT_STATE_TOPIC).c_str(), cJSONString);
		MQTTClient.publish(String(MqttTopic + "/" + MQTT_STATE_TOPIC).c_str(), DataToJSonString(state, AnalogValue, GetSleepDelaySecondsOrDefault()));
	}
}


/**
 * Callback function for the MQTT data
 * 
 * @param topic MQTT topic
 * @param payload Message payload
 * @param length Byte length of the payload
 */ 
void MQTT_Callback(char* topic, byte* payload, unsigned int length){
	char data[length];
	Serial.print("Message arrived in topic: ");
  	Serial.println(topic);

	String SleepTimeTopic = MqttTopic + "/SleepTime";	//build the topic for the sleep time
	String ModeTopic = MqttTopic + "/Mode";				//Build the topic for the operation mode
	
	Serial.print("Message: ");
	for (int i = 0; i < length; i++) {
		Serial.print((char)payload[i]);
		data[i] = (char)payload[i];
	}

	data[length] = '\0';
	Serial.println("");
	Serial.print("Copied data: ");
	Serial.println(data);

	//sensor topic arrived Home/Garden/Sensors/RaisedBed/SleepTime
	if(strcmp(SleepTimeTopic.c_str(), topic)==0){
		Serial.println("MQTT: TOPIC \"SleepTime\" arrived");

		//set line delimiter for casting
		//payload[length] = '\0';

		//cast payload to integer and save as sleep time
		int i_tmp = atoi(data);
		if(i_tmp > 0){
			SaveSleepDelaySeconds(i_tmp);
		}
	}

	//sensor topic arrived Home/Garden/Sensors/RaisedBed/Mode
	if(strcmp(ModeTopic.c_str(), topic)==0){
		Serial.println("MQTT: TOPIC \"Mode\" arrived");

		Serial.print("Payload: ");
		Serial.println(data);
		if(strcmp(data, "AP") == 0)
		{
			RequestStartAP = true;
			Serial.println("Switching to Mode AP");
		}

		if(strcmp(data, "CONFIG") == 0)
		{
			ForceStartWebserver = true;
			RequestNewConfig = true;
			Serial.println("Switching to Mode CONFIG");
		}
	}

	Serial.println();
	Serial.println("--------------------");
}


/**
 * Reads the configured SSID
 * @return SSID or empty if an error occured
 */ 
String GetWifiSSID(){
	delay(100);
	return preferences.getString("SSID", "");
}


/**
 * Writes the SSID to preferences
 * @param SSID SSID to write
 */ 
void SetWiFiSSID(String SSID){
	preferences.putString("SSID", SSID);
}


/**
 * Reads the configured WIFI Key
 * @return Key or empty if an error occured
 */ 
String GetWifiKey(){
	return preferences.getString("WiFiKey", "");
}


/**
 * Writes the WiFI Key to preferences
 * @param Key Key to write
 */ 
void SetWiFiKey(String Key){
	preferences.putString("WiFiKey", Key);
}


/**
 * Read the MQTT username from preferences
 * @return MQTT username
 */ 
String GetMQTTUser(){
	return preferences.getString("MQTTUser", "");
}


/**
 * Saves the MQTT user to preferences
 * @param user user to save
 */ 
void SetMQTTUser(String user){
	preferences.putString("MQTTUser", user);
}



/**
 * Reads the MQTT password from the preferences
 * @return MQTT password
 */ 
String GetMQTTPassword(){
	return preferences.getString("MQTTPwd", "");
}


/**
 * Saves the MQTT password to preferences
 * @param password password to save
 */ 
void SetMQTTPassword(String password){
	preferences.putString("MQTTPwd", password);
}


/**
 * Reads the MQTT broker from the preferences
 */ 
String GetMQTTBroker(){
	return preferences.getString("MQTTBroker", "");
}


/**
 * Saves the MQTT topic to preferences
 */ 
void SetMQTTTopic(String topic){
	//if the topic ends with a /, remove it since it will be added automatically with the /State /mode topics
	unsigned int length = topic.length();
	if(topic.charAt(length-1) == '/')
	{
		Serial.println("Topic before manipulation: ");
		Serial.print(topic);
		topic.remove(length-1);
		Serial.println("Topic after manipulation: ");
		Serial.print(topic);
		Serial.println("");
	}
	preferences.putString("MQTTTopic", topic);
}


/**
 * Reads the MQTT topic from the preferences
 */ 
String GetMQTTTopic(){
	return preferences.getString("MQTTTopic", "");
}


/**
 * Saves the MQTT broker to preferences
 * @param broker the hostname or IP of the broker
 */ 
void SetMQTTBroker(String broker){
	preferences.putString("MQTTBroker", broker);
}


/**
 * Saves the index of the analog sensor input
 * @param index Index to be saved
 */ 
void SetSensorInput(int index)
{
	preferences.putInt("AnalogInput", index);
}


/**
 * Reads the index of the analog sensor input
 * @return the index of the analog input
 */ 
int GetSensorInput()
{
	return preferences.getInt("AnalogInput", -1);
}


/**
 * Saves the index of the trigger output for the sensor
 * @param index Index to be saved
 */ 
void SetTriggerOutput(int index)
{
	preferences.putInt("TriggerOutput", index);
}


/**
 * Reads the index of the trigger output for the sensor
 * @return The index if the digital output
 */ 
int GetTriggerOutput()
{
	return preferences.getInt("TriggerOutput", -1);
}


/**
 * Reads the configuration
 * @return returns if reading the config had an error (e.g. no WIFI or MQTT credentials)
 */
ConfigState ReadConfig(){
	ConfigState ret = CONFIG_OK;
	Serial.println("");
	Serial.println("========== Configuration ==========");

	SleepTimeSeconds = GetSleepDelaySecondsOrDefault();
	WiFi_SSID = GetWifiSSID();
	WiFi_KEY = GetWifiKey();
	MqttBroker = GetMQTTBroker();
	MqttPassword = GetMQTTPassword();
	MqttUser = GetMQTTUser();
	MqttTopic = GetMQTTTopic();
	AnalogInput = GetSensorInput();
	TriggerOutput = GetTriggerOutput();


	Serial.print("Sleeptime in [s]: ");
	Serial.println(SleepTimeSeconds);
	Serial.print("WiFi SSID: ");
	Serial.println(WiFi_SSID);
	Serial.print("WiFI Key: ");
	Serial.println(WiFi_KEY);
	Serial.print("MQTT Broker: ");
	Serial.println(MqttBroker);
	Serial.print("MQTT User: ");
	Serial.println(MqttUser);
	Serial.print("MQTT Password: ");
	Serial.println(MqttPassword);
	Serial.print("MQTT Topic: ");
	Serial.println(MqttTopic);
	Serial.print("Sensor Input: ");
	Serial.println(AnalogInput);
	Serial.print("Trigger Output: ");
	Serial.println(TriggerOutput);


	if((WiFi_SSID.length() == 0) || (WiFi_KEY.length()==0))
	{
		ret = (ConfigState)(ret | WIFI_MISSING);
	}

	if((MqttBroker.length() == 0)||(MqttPassword.length() == 0) || (MqttUser.length() == 0))
	{
		ret	= (ConfigState) (ret | MQTT_MISSING);
	}

	Serial.print("Error in configuration: ");
	Serial.println(ret);

	Serial.println("========== Configuration ==========");
	return ret;
}


/**
 * Opens an access point
 */ 
void StartAP(){
	WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	WiFi.mode(WIFI_AP);
	Serial.println("");
	Serial.println("========== StartAP() ==========");
	Serial.println("Starting Accesspoint");
	WiFi.softAP(AccesPoint_SSID);
	//WiFi.softAPConfig(apIP, apIP, apSubnet);
	Serial.print("IP: ");
	Serial.println(WiFi.softAPIP());
	Serial.println("========== StartAP() ==========");
}


/**
 * starts a watchdog and resets the ESP after the time has elapses
 */ 
void WatchDog(int time)
{

}


/**
 * Saves the data from the Webform to SPIFFS
 * @param AsyncWebServerRequest The request from the Webserver
 */
void SaveWebConfigData(AsyncWebServerRequest *request){
	//delay(100);
	String message;
	
	//save SSID:
	if(request->hasParam("SSID", true)){
		//read the SSID from the POST header
		message = request->getParam("SSID", true)->value();
		//save it to preferences
		SetWiFiSSID(message);
	}

	//save WiFi key
	if(request->hasParam("WIFIKey", true)){
		//read the SSID from the POST header
		message = request->getParam("WIFIKey", true)->value();
		//save it to preferences
		SetWiFiKey(message);
	}

	//save MQTT broker
	if(request->hasParam("MQTTBroker", true)){
		//read the SSID from the POST header
		message = request->getParam("MQTTBroker", true)->value();
		//save it to preferences
		SetMQTTBroker(message);
	}

	//save MQTT user
	if(request->hasParam("MQTTUser", true)){
		//read the SSID from the POST header
		message = request->getParam("MQTTUser", true)->value();
		//save it to preferences
		SetMQTTUser(message);
	}

	//save MQTT password
	if(request->hasParam("MQTTPassword", true)){
		//read the SSID from the POST header
		message = request->getParam("MQTTPassword", true)->value();
		//save it to preferences
		SetMQTTPassword(message);
	}

	//save MQTT topic
	if(request->hasParam("MQTTTopic", true)){
		//read the MQTT topic from the POST header
		message = request->getParam("MQTTTopic", true)->value();
		//save it to preferences
		SetMQTTTopic(message);
	}

	//save analog input
	if(request->hasParam("AnalogInput", true)){
		message = request->getParam("AnalogInput", true)->value();
		SetSensorInput(message.toInt());
	}

	//save trigger output
	if(request->hasParam("TriggerOutput", true)){
		message = request->getParam("TriggerOutput", true)->value();
		SetTriggerOutput(message.toInt());
	}
}


/**
 * Closes all servers/connections and restarts the ESP
 */ 
void Reboot()
{
	Serial.println("Rebooting");
	//report sensor offline to the broker
	if((MQTTClient.connected() == true))
	{
		//MQTTClient.publish(String(MqttTopic + "/" + MQTT_STATE_TOPIC).c_str(), "{\"state\" : \"OFF\"}");
		PublishState(STATE_OFF);
	}
	MQTTClient.disconnect();
	server.end();
	WiFi.disconnect();
	delay(1000);
	ESP.restart();
}


/**
 * Processes the data before they are sent to the webserver.
 * @param var The response with templates from the webserver. See AsyncWebserver documentation
 */ 
String Processor(const String& var){


	if(var == "SSID"){
		return WiFi_SSID;
	}

	if(var == "WIFI_KEY"){
		return WiFi_KEY;
	}

	if(var == "MQTT_BROKER"){
		return MqttBroker;
	}

	if(var == "MQTT_USER"){
		return MqttUser;
	}

	if(var == "MQTT_PASSWORD"){
		return MqttPassword;
	}

	if(var == "MQTT_TOPIC"){
		return MqttTopic;
	}

	if(var == "ANALOG_INPUT"){
		return String(AnalogInput);
	}

	if(var == "TRIGGER_OUTPUT"){
		return String(TriggerOutput);
	}

	if(var == "VERSION"){
		return version;
	}
	return String();
}


void ISR_MQTTCheckTickerElapsed(){
	
	Serial.println("===== ISR_MQTTCheckTickerElapsed() =====");
	Serial.println("Setting flag for rechecking MQTT connection");
	MQTTConnectionCheck = true;
	Serial.println("===== ISR_MQTTCheckTickerElapsed() =====");
}


/**
 * Starts a webserver for configuration
 */ 
void StartWebServer(){
	Serial.println("");
	Serial.println("========== StartWebServer() ==========");
	Serial.println("Starting Webserver");

	dnsServer.start(DNSPort, "*", apIP);

	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    	//request->send(SPIFFS, "/index.html", "text/html", Processor);
		request->send(SPIFFS, "/index.html", String(), false, Processor);
	});

	server.on("/src/bootstrap.bundle.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    	request->send(SPIFFS, "/src/bootstrap.bundle.min.js", "text/javascript");
	});
 
	server.on("/src/jquery-3.5.0.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send(SPIFFS, "/src/jquery-3.5.0.min.js", "text/javascript");
	});
 
	server.on("/src/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send(SPIFFS, "/src/bootstrap.min.css", "text/css");
	});

	server.on("/form_submit", HTTP_POST, [](AsyncWebServerRequest *request){
        SaveWebConfigData(request);
        request->send(200, "text/plain", "Data written, rebooting");
		Reboot();
    });

	server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP

	server.begin();

	//report to MQTT Broker the server state
	if(MQTTClient.connected() == true)
	{
		MQTTClient.publish(String(MqttTopic + "/" + MQTT_STATE_TOPIC).c_str(), "{\"state\" : \"CONFIG\"}");
	}
}


/**
 * Returns the RSSI in dBm as a value of Quality between 0% and 100%
 */ 
uint8_t GetRSSIAsQuality(int8_t RSSIdBm){
	uint8_t quality = 0;

	if(RSSIdBm <= -100){
		quality = 0;
	} else if (RSSIdBm >= -50){
		quality = 100;
	} else {
		quality = uint8_t(2*(RSSIdBm + 100));
	}

	return quality;
}


/**
 * connects to the WIFI using the credentials.
 * @return TRUE if the connection was successful, FALSE if the connection couldn't be established within the defined retries
 */ 
bool ConnectWiFi(){
	Serial.println("");
	Serial.println("===== ConnectWiFi =====");
	Serial.print("SSID: ");
	Serial.println(WiFi_SSID.c_str());
	Serial.print("Key: ");
	Serial.println(WiFi_KEY.c_str());

	

	for(int i = 0; i<MAX_WIFI_CONNECT_RETRIES; i++)
	{
		//during each main loop, RESET WIFI
		WiFi.disconnect();
		WiFi.mode(WIFI_OFF);
		WiFi.mode(WIFI_MODE_STA);
		WiFi.begin(WiFi_SSID.c_str(), WiFi_KEY.c_str());

		//if WIFI connection is successful within the retries, return TRUE
		for(int j = 0; j<WIFI_CONNECT_WAIT_CYCLES; j++)
		{
			if(WiFi.status() == WL_CONNECTED){
				Serial.print("Connected to WiFi. IP: ");
				Serial.println(WiFi.localIP());
				RSSI = GetRSSIAsQuality(WiFi.RSSI());
				Serial.print("WiFi signal strength: ");
				Serial.print(RSSI);
				Serial.println(" %");
				return true;
			}
			delay(500);
			Serial.print(".");
		}
	}

	//if the WIFI connection wasn't successful, return false
	Serial.println("Unable to connect to WIFI");
	return false;
}


/**
 * Connects to the specified MQTT broker
 * @return connection successful
 */ 
bool ConnectMQTT(){
	
	//Build the topic for the MQTT state
	String StateTopic = MqttTopic + "/" + MQTT_STATE_TOPIC;
	String SleepTimeTopic = MqttTopic + "/SleepTime";
	String ModeTopic = MqttTopic + "/Mode";

	MQTTClient.setServer(MqttBroker.c_str(), MQTT_PORT);
	MQTTClient.setCallback(MQTT_Callback);

	for(int i = 0; i<MQTT_CONNECTION_RETRIES; i++)
	{
		//if(MQTTClient.connect(MQTT_CLIENT_ID, MqttUser.c_str(), MqttPassword.c_str(), StateTopic.c_str(), 1, true,"{\"state\" : \"OFF\"}"))
		if(MQTTClient.connect(MQTT_CLIENT_ID, MqttUser.c_str(), MqttPassword.c_str()))
		{
			Serial.println("connected to MQTT broker");
			Serial.print("Topic subscribed: ");
			Serial.println(SleepTimeTopic);
			MQTTClient.subscribe(SleepTimeTopic.c_str());
			Serial.print("Topic subscribed: ");
			Serial.println(ModeTopic);
			MQTTClient.subscribe(ModeTopic.c_str());
			return true;
		}
		delay(10);
	}
	Serial.println("Failed to connect to broker. Reason:");
	Serial.println(MQTTClient.state());
	return false;

}


/**
 * Puts the ESP into deep sleep for the configured time
 */ 
void StartDeepSleep(){
	esp_err_t ret;
	ret = esp_sleep_enable_timer_wakeup(uint64_t(GetSleepDelaySecondsOrDefault()) * uS_TO_S_FACTOR);
	Serial.println("#####");
	Serial.println(esp_err_to_name(ret));
	Serial.println("#####");
	Serial.println("Setup ESP32 to sleep for every " + String(GetSleepDelaySecondsOrDefault()) +
	" Seconds");

	Serial.println("Going to sleep now");
	PublishState(STATE_SLEEP);
	delay(1000);
	Serial.flush(); 

	//before setting the esp to sleep, disconnect MQTT and WIfi and stop preferences
	preferences.end();
	MQTTClient.disconnect();
	WiFi.disconnect();

	esp_deep_sleep_start();

}

/**
 * Interrupt routine when the reboot ticker is elapsed.
 * Restarts the ESP
 */
void ISR_RebootTickerElapsed()
{
	Serial.println("===== ISR_TickerElapsed() =====");
	Serial.println("Setting Flag to true");
	TickerElapsed = true;
	Serial.println("===== ISR_TickerElapsed() =====");
}


/**
 * Setup function
 */ 
void setup(){

	SleepTimeSeconds = 0;
	Serial.begin(9800);
  	delay(1000); //Take some time to open up the Serial Monitor
	DebugBlink(1);
	RequestNewConfig = false;
	RequestStartAP = false;
	preferences.begin("ESPSensor", false);

	//when the ESP ran for MAX_RUNTIME_S, reboot it. It could be that the ESP accidentally
	//came into this state. But even with a webserver started, reboot it since a battery operated
	//device would drain the battery quite fast
	Serial.println("");
	Serial.print("Starting Ticker with ");
	Serial.print(MAX_RUNTIME_S);
	Serial.println(" seconds");
	rebootTicker.attach(MAX_RUNTIME_S, ISR_RebootTickerElapsed);
	

	//initialize SPIFFS
	if(!SPIFFS.begin(true)){
    	Serial.println("An Error has occurred while mounting SPIFFS");
    	return;
  	}

	//initialize state machine
	currentState = START;
	nextState = NONE;
}


/**
 * Program loop
 */ 
void loop(){
	
	if(TickerElapsed == true)
	{
		Reboot();
	}

	//statemachine
	switch (currentState)
	{
	case START:				/* start of the state machine */
		Serial.print("Current state: START - ");
		nextState = READ_CONFIG;
		Serial.println("Next state: READ_CONFIG");
		break;


	case READ_CONFIG:		/* read configuration from preferences */
		Serial.print("Current state: READ_CONFIG - ");
		
		switch (ReadConfig()){
			case CONFIG_OK:	/* reading configuration returned no errors */
				nextState = CONNECT_WIFI;
				Serial.println("Next state: CONNECT_WIFI");
				break;
			case WIFI_MISSING:	/* WIFI data are missing */
				nextState = START_AP;
				Serial.println("Next state: START_AP");
				break;
			case MQTT_MISSING:	/* MQTT data are missing */
				nextState = START_WEBSERVER;
				Serial.println("Next state: START_WEBSERVER");
				break;
			default:	/* e.g. if WIFI and MQTT are missing, start AP */
				nextState = START_AP;
				Serial.println("Next state: START_AP");
				break;
		}
		break;


	case CONNECT_WIFI:		/* establish connection to WiFi */
		Serial.print("Current state: CONNECT_WIFI - ");
		if(ConnectWiFi()){
			nextState = CONNECT_MQTT;
			Serial.println("Next state: CONNECT_MQTT");
		}
		else{
			nextState = START_AP;
			Serial.println("Next state: START_AP");
		}
		break;


	case CONNECT_MQTT:		/* etsablish connection to MQTT broker */
		Serial.print("Current state: CONNECT_MQTT - ");
		if(ConnectMQTT()){
			nextState = OPERATIONAL;
			Serial.println("Next state: OPERATIONAL");
			//execute the MQTT loop in order to get data
			for(int i = 0; i<MAX_MQTT_LOOPS; i++)
			{
				MQTTClient.loop();
				delay(MQTT_LOOP_DELAY_MS);
			}
		}
		else{
			nextState = START_WEBSERVER;
			Serial.println("Next state: START_WEBSERVER");
		}
		break;


	case OPERATIONAL:		/* read subscribed topics, read sensor data and publish */
		Serial.print("Current state: OPERATIONAL - ");
		if(RequestNewConfig){
			nextState = START_WEBSERVER;
			MQTTClient.publish(String(MqttTopic + "/Mode").c_str(), "", true);	//clear the retained topic
			Serial.println("Next state: READ_CONFIG");
		}
		else if(RequestStartAP){
			nextState = START_AP;
			Serial.println("Next state: START_AP");
			MQTTClient.publish(String(MqttTopic + "/Mode").c_str(), "", true);	//clear the retained topic
		}
		else if(CyclicSensorReading){
			nextState = READ_SENSOR;
			Serial.println("Start cyclic reading of sensor data");
		}
		else{
			//read and store the analog value
			AnalogValue = ReadAnalogValue(AnalogInput, TriggerOutput);
			PublishState(STATE_ON);
			nextState = DEEP_SLEEP;
			Serial.println("Next state: DEEP_SLEEP");
		}
		break;


	case DEEP_SLEEP:		/* set the ESP to deep sleep */
		StartDeepSleep();
		break;


	case START_AP:			/* start wifi access point */
		Serial.print("Current state: START_AP - ");
		nextState = START_WEBSERVER;
		APMode = true;
		StartAP();
		Serial.println("Next state: START_WEBSERVER");
		break;


	case START_WEBSERVER:	/* start webserver with configuration interface */
		Serial.println("Current state: START_WEBSERVER");
		nextState = WEBSERVER_RUNNING;

		//if the webserver was started accidentally (be sporadic loosing pf MQTT connection) recheck the MQTT connection each 
		//WEBSERVER_MQTT_RECHECK_TIME_S. If the connection was successful, the ESP restarts
		if(ForceStartWebserver == false){
			MQTTRecheckTicker.attach(WEBSERVER_MQTT_RECHECK_TIME_S, ISR_MQTTCheckTickerElapsed);
		}		
		StartWebServer();
		Serial.println("Next state: WEBSERVER_RUNNING");
		break;

	case WEBSERVER_RUNNING:
		dnsServer.processNextRequest();

		//if the ticker for rechecking the MQTT connection is elapsed and we are NOT in AP mode
		if(MQTTConnectionCheck == true && APMode == false){
			MQTTConnectionCheck = false;
			//In webserver mode we want to reboot if the MQTT connection was successful. 
			//if the ESP wakes up and couldn't connect to MQTT it (sporadically) make sure that
			//it rechecks the connection and then reboots. The default time is WEBSERVER_MQTT_RECHECK_TIME_S
			if(ConnectMQTT())
			{
				MQTTRecheckTicker.detach();
				Reboot();
			}
		}
		break;

		
	case ERROR:				/* error happened. Currently not used */
	case NONE:				/* currently not used */
	case READ_SENSOR:
		Serial.println(ReadAnalogValue(AnalogInput, TriggerOutput));
		delay(1000);
		nextState = READ_SENSOR;
		break;
	default:				/* restart ESP */
		Reboot();
		break;
	}

	//switch states
	currentState = nextState;
}


