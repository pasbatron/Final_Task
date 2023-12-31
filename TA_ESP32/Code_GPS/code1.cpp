#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <WiFi.h>



const char* ssid = "Pena";
const char* password = "wandaadib";
const char* username = "wanda";
const char* pubTopic = "sahdfkashdf_publish_gps_lat";
const unsigned int writeInterval = 25000;

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
const char* mqtt_server = "44.195.202.69";
unsigned int mqtt_port = 1883;


// objects
WiFiClient askClient;
PubSubClient client(askClient);
TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device



// GPS displayInfo
void displayInfo() {
if (gps.location.isValid()) {
double latitude = (gps.location.lat());
double longitude = (gps.location.lng());
Serial.println("********** Publish MQTT data to ASKSENSORS");
char mqtt_payload[50] = "";
snprintf (mqtt_payload, 50, "m1=%lf;%lf", latitude, longitude);
Serial.print("Publish message: ");
Serial.println(mqtt_payload);
client.publish(pubTopic, mqtt_payload);
Serial.println("> MQTT data published");
Serial.println("********** End ");
Serial.println("*****************************************************");
delay(writeInterval);// delay 
} else {
    Serial.println(F("INVALID"));
  }

}



//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");
for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  } 
  Serial.println();
}



//MQTT reconnect
void reconnect() {
// Loop until we're reconnected
while (!client.connected()) {
Serial.print("********** Attempting MQTT connection...");
// Attempt to connect
if (client.connect("ESP32Client", username, "")) { 
  Serial.println("-> MQTT client connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println("-> try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
    }
  }
}




// setup
void setup() {

    Serial.begin(115200);
    Serial.println("*****************************************************");
    Serial.println("********** Program Start : ESP32 publishes NEO-6M GPS position to AskSensors over MQTT");
    Serial.print("********** connecting to WIFI : ");
    Serial.println(ssid);


    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("->WiFi connected");
    Serial.println("->IP address: ");
    Serial.println(WiFi.localIP());

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    // GPS baud rate
    ss.begin(GPSBaud);

}


// loop
void loop() {
    if (!client.connected()) 
    reconnect();
    client.loop();

    // This sketch displays information every time a new sentence is correctly encoded.

    while (ss.available() > 0)
    if (gps.encode(ss.read()))

    displayInfo();

    if (millis() > 5000 && gps.charsProcessed() < 10){
        Serial.println(F("No GPS detected: check wiring."));
        while(true);
    }
}


