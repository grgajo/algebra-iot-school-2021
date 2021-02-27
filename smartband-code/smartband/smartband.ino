/**
 *       ALGEBRA IoT school   
 *
 * @file       smartband.ino
 *
 * @subsystem  
 *
 * @brief      Basic implementation of patient monitoring tracker functionality. 
 *
 * @author     Tomislav Razov    SmartSense, 2021
 *				
 *			   Oximeter code taken from MAXREFDES117 example program by Robert Fraczkiewicz
 *				
 *			   Operation: measures temperature and battery voltage every few seconds and sends 
 *						  them to MQTT broker. When finger is placed on SpO2 sensor and held still,
 *						  pulse will be measured and sent. Red LED will indicate successfull pulse 
 *						  readout. 
 *
 */


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "Adafruit_MCP9808.h"

#include <WiFi.h>
#include <PubSubClient.h>

//==================================================
// !!!IMPORTANT: CHANGE THIS TO YOUR DATA!!!
const char* ssid  = "Algebra-HotSpot";
const char* pass = "";
const String student_name("grga");
const char* mqtt_server = "213.191.133.132";
const int   mqtt_port = 1883;
//==================================================

// MQTT topics strings
const String temperature_topic = String("algebra/iot/") + student_name + String("/temperature");
const String battery_topic = String("algebra/iot/") + student_name + String("/battery");
const String pulse_topic = String("algebra/iot/") + student_name + String("/pulse");
const String command_topic = String("algebra/iot/") + student_name + String("/command");


unsigned long mqtt_start_time;

WiFiClient client;
PubSubClient mqttClient(client);

//#define DEBUG // Uncomment for debug output to the Serial stream
//#define TEST_MAXIM_ALGORITHM // Uncomment if you want to include results returned by the original MAXIM algorithm
//#define SAVE_RAW_DATA // Uncomment if you want raw data coming out of the sensor saved to SD card. Red signal first, IR second.


#ifdef TEST_MAXIM_ALGORITHM
  #include "algorithm.h" 
#endif

// Interrupt pin
const byte oxiInt = 14; 	// pin connected to MAX30102 INT
const byte ledPin = 13; 	// LED PIN
const byte vbattPin = 35;	// VBAT voltage

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;


void read_temperature();


void connect_to_wifi() {
  
  Serial.println("Connecting to WiFi");
  
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("received IP address: ");
  Serial.println(WiFi.localIP());
}


void power_off_sensors() {
	
	Serial.println("Shutting down MSP9808...");
	tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
	Serial.println("Shutting down MAX30102...");
	maxim_max30102_reset();
}


// Callback used for MQTT subscriber client
void callback(char* topic, byte* payload, unsigned int length) {

	char buf [32];

	Serial.println("message received");
	Serial.print("  -> topic: ");
	Serial.println(topic);

	Serial.print("  -> message: ");

	int i;
  
	for (i = 0; i < length; i++) {
    
		Serial.print((char)payload[i]);
		buf[i] = (char)payload[i];
	}
  
	buf[i+1] = 0; //nul termination for string
	Serial.println();


	if (strncmp("sleep", buf, strlen("sleep")) == 0) {

		mqttClient.publish("algebra/iot/tomo/status", "sleeping");
	}

	// go to power saving mode, you can wake the bracelet using "reset" button
	power_off_sensors();
  
	//hasta la vista bejbe!
	esp_deep_sleep_start();
}



void setup() {

	pinMode(oxiInt, INPUT);  	// pin 14 connects to the interrupt output pin of the MAX30102
	pinMode(vbattPin, INPUT);	// ADC to measure battery voltage
	pinMode(ledPin, OUTPUT);	// RED LED on standard PIN 13 - indicates successfull pulse measurement
	
	mqtt_start_time = millis();

	Wire.begin();
  
	Serial.begin(115200);
	
	mqttClient.setServer(mqtt_server, mqtt_port);
	mqttClient.setCallback(callback);
  
	if (!tempsensor.begin(0x18)) {
    
		Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
		
	}
	else {
	
		Serial.println("Found MCP9808!");
		tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
		// Mode Resolution SampleTime
		//  0    0.5°C       30 ms
		//  1    0.25°C      65 ms
		//  2    0.125°C     130 ms
		//  3    0.0625°C    250 ms
	
	
		Serial.println("wake up MCP9808.... "); // wake up MCP9808 - power consumption ~200 mikro Ampere
		tempsensor.wake();   // wake up, ready to read
	}
   
	Serial.println("Initializing MAX30102...");
	maxim_max30102_reset(); //resets the MAX30102
	delay(1000);

	maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
	maxim_max30102_init();  //initialize the MAX30102
	old_n_spo2=0.0;


#ifdef TEST_MAXIM_ALGORITHM
	Serial.print(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr"));
#else // TEST_MAXIM_ALGORITHM
	Serial.print(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr"));
#endif // TEST_MAXIM_ALGORITHM
#ifdef SAVE_RAW_DATA
	int32_t i;
	
	// These are headers for the red signal
	for(i=0;i<BUFFER_SIZE;++i) {
		Serial.print("\t");
		Serial.print(i);
	}
	// These are headers for the infrared signal
	for(i=0;i<BUFFER_SIZE;++i) {
		Serial.print("\t");
		Serial.print(i);
	}
	
#endif // SAVE_RAW_DATA
	Serial.println("");
  
	timeStart=millis();
	
	connect_to_wifi();
}


// here we take care that our connection to MQTT broker is active and we maintain our subscription
void mqtt_handler() {

	static int count = 0;

	// We check the MQTT connection every 5 seconds  
	if (millis() - mqtt_start_time >= 5000) {

		mqtt_start_time = millis();
    
		if (!mqttClient.connected()) {

			if (mqttClient.connect((String("narukvca_") + student_name).c_str())) {
          
				mqttClient.subscribe(command_topic.c_str());
          
				Serial.println("Connected to mqtt broker");

			}
			else {
        
				Serial.println("Failed to connect to mqtt broker");
			} 
		}
	} 
  
	mqttClient.loop(); 
}


void read_temperature() {

	float c = tempsensor.readTempC();
	Serial.print("Temp: "); 
	Serial.print(c, 4); Serial.println(" oC"); 
	
	mqttClient.publish(temperature_topic.c_str(), String(c).c_str());
	
}

void measure_batt_voltage() {
	
	uint16_t voltage = analogRead(vbattPin);
	Serial.print("Vbatt: "); 
	Serial.print(voltage * 2); 
	Serial.println(" mV");
	
	mqttClient.publish(battery_topic.c_str(), String(voltage * 2).c_str());
}



//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
	
	float n_spo2,ratio,correl;  //SPO2 value
	int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
	int32_t n_heart_rate; //heart rate value
	int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
	int32_t i;
	char hr_str[10];
     
	//buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
	//read BUFFER_SIZE samples, and determine the signal range
  
	read_temperature();
	measure_batt_voltage();
  
	for(i=0;i<BUFFER_SIZE;i++) {
		
		while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
		maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO

#ifdef DEBUG
		Serial.print(i, DEC);
		Serial.print(F("\t"));
		Serial.print(aun_red_buffer[i], DEC);
		Serial.print(F("\t"));
		Serial.print(aun_ir_buffer[i], DEC);    
		Serial.println("");
#endif // DEBUG
	}

	
	//calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
	rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
	
	elapsedTime=millis()-timeStart;
	
	millis_to_hours(elapsedTime,hr_str); // Time in hh:mm:ss format
	elapsedTime/=1000; // Time in seconds

#ifdef DEBUG
	Serial.println("--RF--");
	Serial.print(elapsedTime);
	Serial.print("\t");
	Serial.print(n_spo2);
	Serial.print("\t");
	Serial.print(n_heart_rate, DEC);
	Serial.print("\t");
	Serial.println(hr_str);
	Serial.println("------");
#endif // DEBUG

#ifdef TEST_MAXIM_ALGORITHM
  
	//calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using MAXIM's method
	float n_spo2_maxim;  //SPO2 value
	int8_t ch_spo2_valid_maxim;  //indicator to show if the SPO2 calculation is valid
	int32_t n_heart_rate_maxim; //heart rate value
	int8_t  ch_hr_valid_maxim;  //indicator to show if the heart rate calculation is valid
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2_maxim, &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim); 

#ifdef DEBUG
	Serial.println("--MX--");
	Serial.print(elapsedTime);
	Serial.print("\t");
	Serial.print(n_spo2_maxim);
	Serial.print("\t");
	Serial.print(n_heart_rate_maxim, DEC);
	Serial.print("\t");
	Serial.println(hr_str);
	Serial.println("------");
#endif // DEBUG
#endif // TEST_MAXIM_ALGORITHM

  //save samples and calculation result to SD card
#ifdef TEST_MAXIM_ALGORITHM
	if((ch_hr_valid && ch_spo2_valid) || (ch_hr_valid_maxim && ch_spo2_valid_maxim)) {
#else   // TEST_MAXIM_ALGORITHM
	if(ch_hr_valid && ch_spo2_valid) { 
#endif // TEST_MAXIM_ALGORITHM

		//signal we have a valid pulse readout
		digitalWrite(ledPin, HIGH);

		Serial.print(elapsedTime);
		Serial.print("\t");
		Serial.print(n_spo2);
		Serial.print("\t");
		Serial.print(n_heart_rate, DEC);
		Serial.print("\t");
		mqttClient.publish(pulse_topic.c_str(), String(n_heart_rate).c_str());
#ifdef TEST_MAXIM_ALGORITHM
		Serial.print(n_spo2_maxim);
		Serial.print("\t");
		Serial.print(n_heart_rate_maxim, DEC);
		Serial.print("\t");
#endif //TEST_MAXIM_ALGORITHM
		Serial.print(hr_str);
		Serial.print("\t");
		Serial.print(ratio);
		Serial.print("\t");
		Serial.print(correl);
#ifdef SAVE_RAW_DATA
    
		// Save raw data for unusual O2 levels
		for(i=0;i<BUFFER_SIZE;++i) {
		
			Serial.print(F("\t"));
			Serial.print(aun_red_buffer[i], DEC);
		}
		
		for(i=0;i<BUFFER_SIZE;++i) {
			
			Serial.print(F("\t"));
			Serial.print(aun_ir_buffer[i], DEC);    
		}

#endif // SAVE_RAW_DATA
		Serial.println("");
		old_n_spo2=n_spo2;
	}
	else {
	  
		digitalWrite(ledPin, LOW); 
	}
  
	mqtt_handler();
}



void millis_to_hours(uint32_t ms, char* hr_str) {
	
	char istr[6];
	uint32_t secs,mins,hrs;
	secs=ms/1000; // time in seconds
	mins=secs/60; // time in minutes
	secs-=60*mins; // leftover seconds
	hrs=mins/60; // time in hours
	mins-=60*hrs; // leftover minutes
	itoa(hrs,hr_str,10);
	strcat(hr_str,":");
	itoa(mins,istr,10);
	strcat(hr_str,istr);
	strcat(hr_str,":");
	itoa(secs,istr,10);
	strcat(hr_str,istr);
}
