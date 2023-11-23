//libraries to integrate functionality
#include <WiFi.h> //wifi connection
#include <PubSubClient.h> //MQTT messaging
#include <Arduino.h> //Input and output

//Libraries for temp sensor
#include <OneWire.h>
#include <DallasTemperature.h>  

//Libraries for Ultrasonic
#include "NewPing.h"
#include "Arduino.h"


//Definitions for TDS
#define TdsSensorPin 32
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

//Pins for ultrasonic
#define trigPin  26 //Pin that is connected to the Trigger pin
#define echoPin  27 //Pin that is connected to the Echo pin

//Inputs
#define SENSOR_PIN  17 // ESP32 pin GPIO17 connected to DS18B20 sensor's DATA pin
int lightpin = 34; // Analog pin connected to the photocell
int lightValue = 0; // Variable to store the light level

//Connects to wife---------------------------------------------------------------------------------------------------------
// Wi-Fi credentials: replace with those of your network
const char* ssid = "BELL419";  // The name of the WiFi network
const char* password = "46193941191D"; // The WiFi network passkey
//Connects to MQTT Broker
// MQTT broker details: replace with your own
const char* mqtt_server = "chaveropi.local"; // The MQTT broker's hostname or IP address
const int mqtt_port = 1883;  // MQTT broker port (1883 is default)
const char* mqtt_topic = "tank/temp";  // MQTT topic to publish messages
// MQTT client name prefix (will add MAC address)
String name = "ESP32Client_";
// Create an instance of the WiFiClient class
WiFiClient espClient;
// Create an instance of the PubSubClient class
PubSubClient client(espClient);
//-------------------------------------------------------------------------------------------------------------------------

// Timer for publishing every 5 seconds
unsigned long previousMillis = 0;
const long interval = 1000;

OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

//Variables for temp
float tempC; // temperature in Celsius

//Variables for TDS
int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

//Variables for Ultrasonic
float distance;
float duration;

// median filtering algorithm for TDS sensor
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


void setup() {
  // Start Serial communication
  Serial.begin(115200);
  DS18B20.begin();    // initialize the DS18B20 sensor
  
  //Define pins as inputs
  pinMode(lightpin, INPUT);
  pinMode(TdsSensorPin,INPUT);

  pinMode(trigPin, OUTPUT); //define trigger pin as an output
  pinMode(echoPin, INPUT); //define echo pin as an input
  
  // Read the MAC address------------------------------------------------------------
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  // Convert MAC address to a string
  char macStr[18]; // MAC address is 12 characters long without separators, plus null terminator
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // Concatenate the name prefix with the MAC address 
  name = name + macStr;
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  //---------------------------------------------------------------------------------

  // Set MQTT server and port
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  //Temp Sensor-------------------------------------------------------
  DS18B20.requestTemperatures();       // send the command to get temperatures
  tempC = DS18B20.getTempCByIndex(0);  // read temperature in °C
  String tempMessage = String(tempC);
  client.publish("water/temp", tempMessage.c_str());

  //TDS Sensor---------------------------------------------------------
static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      String tdsMessage = String(tdsValue);
      client.publish("water/quality", tdsMessage.c_str());
      /*Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");*/
    }
  }
  //---------------------------------------------------------
  //Ultrasonic
  // put your main code here, to run repeatedly:
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //sends a pulse with the trigger pin 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); //times the duration of the pulse 
  distance = (duration/2)*0.0343;  //calculates the distance with the duration read
  String lvlMessage = String(distance);
  client.publish("water/level", lvlMessage.c_str());
  //--------------------------------------------------------

  // Connect to MQTT if necessary
  if (!client.connected()) {
    connect();
  }

  // Get the current time
  unsigned long currentMillis = millis();

  // Publish a message every second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    lightValue = analogRead(lightpin);
  // Publish the data to the MQTT topic
  String lightMessage = String(lightValue);
  client.publish("light/level", lightMessage.c_str());
  }

  // Allow the PubSubClient to process incoming messages
  client.loop();
}

void connect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(name.c_str())) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 5 seconds");
      delay(5000);
    }
  }
}
