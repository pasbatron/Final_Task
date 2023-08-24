#include <Arduino.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <WiFi.h>


const char* ssid = "Pena";
const char* password = "wandaadi";
const char* mqtt_server = "44.195.202.69";
const unsigned int writeInterval = 25000;
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
const int pinLed = 13;


float gyro_akselerasi_x;
float gyro_akselerasi_y;
float gyro_akselerasi_z;
float gyro_rotasi_x;
float gyro_rotasi_y;
float gyro_rotasi_z;
float gyro_temp;
float gps_latitude;
float gps_longitude;
float gps_altitude;


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
  if ((char)payload[0] == '1') {
    digitalWrite(pinLed, HIGH);
  }
  if ((char)payload[0] == '0') {
    digitalWrite(pinLed, LOW);
  }
}





void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("wanda/esp32/v1", "hello world");
      client.subscribe("wanda/esp32/v1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



void displayInfo(){
  if (gps.location.isValid()) {
  double latitude = (gps.location.lat());
  double longitude = (gps.location.lng());

  Serial.println("********** Publish MQTT data to ASKSENSORS");
  char mqtt_payload[50] = "";
  snprintf (mqtt_payload, 50, "m1=%lf;%lf", latitude, longitude);
  Serial.print("Publish message: ");
  Serial.println(mqtt_payload);
  Serial.println("> MQTT data published");
  Serial.println("********** End ");
  Serial.println("*****************************************************");

  delay(writeInterval);// delay 
  } else {
    Serial.println(F("INVALID"));
  }

}



void setup() {
  pinMode(pinLed, OUTPUT);
  Serial.begin(115200);
  ss.begin(GPSBaud);

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }


  display.display();


  delay(500);
  setup_wifi();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


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


void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  Serial.println("");
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      displayInfo();
    }
  }
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("wanda/esp32/v1", msg);

    gyro_akselerasi_x = a.acceleration.x;
    char akselerasi_x_String[8];
    dtostrf(gyro_akselerasi_x, 1, 2, akselerasi_x_String);
    Serial.print("gyro_akselerasi_y: ");
    Serial.println(akselerasi_x_String);
    client.publish("lkasdfnaskjdfbka_gyro_akselerasi_x", akselerasi_x_String);

    gyro_akselerasi_y = a.acceleration.y;
    char akselerasi_y_String[8];
    dtostrf(gyro_akselerasi_y, 1, 2, akselerasi_y_String);
    Serial.print("gyro_akselerasi_y: ");
    Serial.println(akselerasi_y_String);
    client.publish("lkasdfnaskjdfbka_gyro_akselerasi_y", akselerasi_y_String);

    gyro_akselerasi_z = a.acceleration.z;
    char akselerasi_z_String[8];
    dtostrf(gyro_akselerasi_z, 1, 2, akselerasi_z_String);
    Serial.print("gyro_akselerasi_z: ");
    Serial.println(akselerasi_z_String);
    client.publish("lkasdfnaskjdfbka_gyro_akselerasi_z", akselerasi_z_String);

    gyro_rotasi_x = g.gyro.x;
    char rotasi_x_String[8];
    dtostrf(gyro_rotasi_x, 1, 2, rotasi_x_String);
    Serial.print("gyro_rotasi_x: ");
    Serial.println(rotasi_x_String);
    client.publish("lkasdfnaskjdfbka_gyro_rotasi_x", rotasi_x_String);

    gyro_rotasi_y = g.gyro.y;
    char rotasi_y_String[8];
    dtostrf(gyro_rotasi_y, 1, 2, rotasi_y_String);
    Serial.print("gyro_rotasi_y: ");
    Serial.println(rotasi_y_String);
    client.publish("lkasdfnaskjdfbka_gyro_rotasi_y", rotasi_y_String);

    gyro_rotasi_z = g.gyro.z;
    char rotasi_z_String[8];
    dtostrf(gyro_rotasi_z, 1, 2, rotasi_z_String);
    Serial.print("gyro_rotasi_z: ");
    Serial.println(rotasi_z_String);
    client.publish("lkasdfnaskjdfbka_gyro_rotasi_z", rotasi_z_String);

    gyro_temp = temp.temperature;
    char temp_String[8];
    dtostrf(gyro_temp, 1, 2, temp_String);
    Serial.print("gyro_temp : ");
    Serial.println(temp_String);
    client.publish("lkasdfnaskjdfbka_gyro_temp", temp_String);

    gps_longitude = gps.location.lat(), 6;
    char gps_longitude_String[8];
    dtostrf(gps_longitude, 1, 2, gps_longitude_String);
    Serial.print("gps_long : ");
    Serial.println(gps_longitude_String);
    client.publish("lkasdfnaskjdfbka_gps_long", gps_longitude_String);

    gps_latitude = gps.location.lng(), 6;
    char gps_latitude_String[8];
    dtostrf(gps_latitude, 1, 2, gps_latitude_String);
    Serial.print("gps_lat : ");
    Serial.println(gps_latitude_String);
    client.publish("lkasdfnaskjdfbka_gps_lat", gps_latitude_String);

    Serial.print(gps.location.lat(), 6);

    // ________________________________________________________________
    // kode untuk altitude, kecepatan, gradien

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Accelerometer - m/s^2");
    display.print(a.acceleration.x, 1);
    display.print(", ");
    display.print(a.acceleration.y, 1);
    display.print(", ");
    display.print(a.acceleration.z, 1);
    display.println("");

    display.println("Gyroscope - rps");
    display.print(g.gyro.x, 1);
    display.print(", ");
    display.print(g.gyro.y, 1);
    display.print(", ");
    display.print(g.gyro.z, 1);
    display.println("");

    display.println("Latitude,longitude: ");
    display.print(gps.location.lat(), 1);
    display.print(", ");
    display.print(gps.location.lng(), 1);
    display.println("");
    display.display();
    }
  delay(5000);
}
