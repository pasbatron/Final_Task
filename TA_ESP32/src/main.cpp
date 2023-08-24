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
const char* pub_topic_lat = "publish_e_v_ta_latitude"; // publish/username/apiKeyIn
const char* pub_topic_long = "publish_e_v_ta_longitude";
const char* pub_topic_alt = "publish_e_v_ta_altitude";
const char* pub_topic_aks_x = "publish_e_v_ta_gyro_akselerasi_x";
const char* pub_topic_aks_y = "publish_e_v_ta_gyro_akselerasi_y";
const char* pub_topic_aks_z = "publish_e_v_ta_gyro_akselerasi_z";
const char* pub_topic_rts_x = "publish_e_v_ta_gyro_rotasi_x";
const char* pub_topic_rts_y = "publish_e_v_ta_gyro_rotasi_y";
const char* pub_topic_rts_z = "publish_e_v_ta_gyro_rotasi_z";
const char* pub_topic_temp = "publish_e_v_ta_gyro_temperature";



const unsigned int writeInterval = 25000;
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
const char* mqtt_server = "broker.mqtt-dashboard.com";
unsigned int mqtt_port = 1883;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;


Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient client(espClient);
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");
for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  } 
  Serial.println();
}

void main_gyro(){
  while (!Serial)
    delay(10);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("");
  delay(100);
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("outTopic", "hello world");
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



void control_program() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (gps.location.isValid()){

    double latitude = (gps.location.lat());
    double longitude = (gps.location.lng());
    double altitude = (gps.altitude.value());
    double akselerasi_x = a.acceleration.x;
    double akselerasi_y = a.acceleration.y;
    double akselerasi_z = a.acceleration.z;
    double rotasi_x = g.gyro.x;
    double rotasi_y = g.gyro.y;
    double rotasi_z = g.gyro.z;
    double temp_gyro = temp.temperature;

    char mqtt_payload_gps_longitude[50] = "";
    char mqtt_payload_gps_latitude[50] = "";
    char mqtt_payload_gps_altitude[50] = "";
    char mqtt_payload_akselerasi_x[50] = "";
    char mqtt_payload_akselerasi_y[50] = "";
    char mqtt_payload_akselerasi_z[50] = "";
    char mqtt_payload_rotasi_x[50] = "";
    char mqtt_payload_rotasi_y[50] = "";
    char mqtt_payload_rotasi_z[50] = "";
    char mqtt_payload_temp[50] = "";


    snprintf (mqtt_payload_gps_latitude, 50, "%lf", latitude);
    Serial.print("Publish message gps-lat: ");
    Serial.println(mqtt_payload_gps_latitude);
    client.publish(pub_topic_lat, mqtt_payload_gps_latitude);

    snprintf (mqtt_payload_gps_longitude, 50, "%lf", longitude);
    Serial.print("Publish message gps-long: ");
    Serial.println(mqtt_payload_gps_longitude);
    client.publish(pub_topic_long, mqtt_payload_gps_longitude);

    snprintf (mqtt_payload_gps_altitude, 50, "%lf", altitude);
    Serial.print("Publish message gps-alt: ");
    Serial.println(mqtt_payload_gps_altitude);
    client.publish(pub_topic_alt, mqtt_payload_gps_altitude);

    snprintf (mqtt_payload_akselerasi_x, 50, "%lf", akselerasi_x);
    Serial.print("Publish message akselerasi-x: ");
    Serial.println(mqtt_payload_akselerasi_x);
    client.publish(pub_topic_aks_x, mqtt_payload_akselerasi_x);

    snprintf (mqtt_payload_akselerasi_y, 50, "%lf", akselerasi_y);
    Serial.print("Publish message akselerasi-y: ");
    Serial.println(mqtt_payload_akselerasi_y);
    client.publish(pub_topic_aks_y, mqtt_payload_akselerasi_y);

    snprintf (mqtt_payload_akselerasi_z, 50, "%lf", akselerasi_z);
    Serial.print("Publish message akselerasi-z: ");
    Serial.println(mqtt_payload_akselerasi_z);
    client.publish(pub_topic_aks_z, mqtt_payload_akselerasi_z);


    snprintf (mqtt_payload_rotasi_x, 50, "%lf", rotasi_x);
    Serial.print("Publish message rotasi-x: ");
    Serial.println(mqtt_payload_rotasi_x);
    client.publish(pub_topic_rts_x, mqtt_payload_rotasi_x);


    snprintf (mqtt_payload_rotasi_y, 50, "%lf", rotasi_y);
    Serial.print("Publish message rotasi-y: ");
    Serial.println(mqtt_payload_rotasi_y);
    client.publish(pub_topic_rts_y, mqtt_payload_rotasi_y);

    snprintf (mqtt_payload_rotasi_z, 50, "%lf", rotasi_z);
    Serial.print("Publish message rotasi-z: ");
    Serial.println(mqtt_payload_rotasi_z);
    client.publish(pub_topic_rts_z, mqtt_payload_rotasi_z);

    snprintf (mqtt_payload_temp, 50, "%lf", temp_gyro);
    Serial.print("Publish message temp-gyro: ");
    Serial.println(mqtt_payload_temp);
    client.publish(pub_topic_temp, mqtt_payload_temp);

    Serial.println("> MQTT data published");

// _______________________________________________
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Monitoring DAQ-Wnd");
    display.println("===================");
    display.println("Lat,long,alt");
    display.print(latitude, 1);
    display.print(", ");
    display.print(longitude, 1);
    display.print(", ");
    display.print(altitude, 1);
    display.println("");
    display.display();
    display.println("Akselerasi:x,y,z");
    display.print(akselerasi_x, 1);
    display.print(", ");
    display.print(akselerasi_y, 1);
    display.print(", ");
    display.print(akselerasi_z, 1);
    display.println("");
    display.display();
    display.println("Rotasi:x,y,z");
    display.print(rotasi_x, 1);
    display.print(", ");
    display.print(rotasi_y, 1);
    display.print(", ");
    display.print(rotasi_z, 1);
    display.println("");
    display.display();
    delay(writeInterval);// delay
  }
  else{
    Serial.println(F("INVALID"));
  }
}


// setup
void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    ss.begin(GPSBaud);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
      Serial.println(F("SSD1306 allocation failed"));
      for (;;)
        ;
    }
    delay(500);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setRotation(0);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    if (!mpu.begin()) {
      Serial.println("Sensor init failed");
      while (1)
        yield();
    }
    main_gyro();
}
void loop() {
    if (!client.connected()) 
    reconnect();
    client.loop();
    while (ss.available() > 0)
    if(gps.encode(ss.read()))
    control_program();
    if (millis() > 5000 && gps.charsProcessed() < 10){
        Serial.println(F("No GPS detected: check wiring."));
        while(true);
    }

}


