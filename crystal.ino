#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>  
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>


const char* ssid = "Yoo";       
const char* password = "tanishk1234"; 
const char* broker_ip = "10.125.32.63"; 
const int broker_port = 1883;
#define CLIENT_ID "ESPClient1"
#define CMD_TOPIC "esp/cmd/1"
#define SENSOR_TOPIC "esp/sensor/1"
#define LED_TOPIC "esp/led"
#define MOS_TOPIC "esp/mos"
Servo servo1; 
Servo servo2; 
Servo servo3; 
Servo elbow;
Servo base;
// Defining pins
#define servo1_pin  27
#define servo2_pin  26
#define servo3_pin  25
#define base_pin   33
#define elbow_pin    32
const int SOL_PIN = 23;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 1000;    // 1 kHz
const int PWM_RES = 8;

const int PULLIN_MS = 300;   
const int HOLD_MS = 2000;  
const int OFF_MS = 1000;     
const int HOLD_DUTY = 255;
const int PWM_NEUTRAL = 1495;
const int PWM_MIN = 845;
const int PWM_MAX = 2145;

bool mos_status = false;
float m1,m2,m3,elbow_angle,base_angle;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void servo_init(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);    // Standard 50hz servo
  servo1.attach(servo1_pin, 500, 2400); // Attach with min/max pulses
  
  servo2.setPeriodHertz(50);
  servo2.attach(servo2_pin, 500, 2400);

  servo3.setPeriodHertz(50);
  servo3.attach(servo3_pin, 500, 2400);

  elbow.setPeriodHertz(50);
  elbow.attach(elbow_pin, 500, 2400);

  base.setPeriodHertz(50);
  base.attach(base_pin,500,2400);
}

void setup_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {  // ~10s timeout
        Serial.println(F("Waiting for hotspot..."));
        delay(500);
        attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi connection failed! Restarting..."));
        ESP.restart();  // Restart on failure
    }
    Serial.println(F("WiFi connected."));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);

    char message_str[length + 1];
    memcpy(message_str, payload, length);
    message_str[length] = '\0';
    String message = String(message_str);
    
    // --- CASE 2: COMMAND TOPIC (JSON Data) ---
   if (strcmp(topic, CMD_TOPIC) == 0) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, message);

        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        m1 = doc["m1"];
        m2 = doc["m2"];
        m3 = doc["m3"];
        base_angle = doc["arm"]["base"];
        elbow_angle = doc["arm"]["elbow"];
        mos_status = doc["mos"];

        m1 = vel_to_pwm(m1);
        m2 = vel_to_pwm(m2);
        m3 = vel_to_pwm(m3);

        // Command Servos
        servo1.write(m1);
        servo2.write(m2);
        servo3.write(m3);
        
        elbow.write(elbow_angle);
        base.write(base_angle);

  }
  if (strcmp(topic, MOS_TOPIC) == 0){
    JsonDocument mos_doc;
    DeserializationError error = deserializeJson(mos_doc, message);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    //extracting values
    mos_status = mos_doc["status"];
    if (mos_status == 1){
      ledcWrite(SOL_PIN, 255); // full power
      ledcWrite(SOL_PIN, HOLD_DUTY); // reduced hold duty
    }
    else if(mos_status == 0){
      ledcWrite(SOL_PIN, 0);
    }
  }
    


}

float vel_to_pwm (float vel){
  vel = max(-1.0f, min(1.0f, vel));
  if (abs(vel) < 0.05f){
      return PWM_NEUTRAL;
  }
  if (vel > 0){
    return float(PWM_NEUTRAL + vel * (PWM_MAX - PWM_NEUTRAL));

  }
  else{
    return float(PWM_NEUTRAL + vel * (PWM_NEUTRAL - PWM_MIN));
  }
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(CLIENT_ID)) {
            Serial.println("connected");
            mqttClient.subscribe(CMD_TOPIC);
            mqttClient.subscribe(LED_TOPIC);
            mqttClient.subscribe(MOS_TOPIC);

            Serial.println(F("Subscribed to command topic."));
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // delay(5000);
        }
    }
}

void publishSensorData() {
    float proximity = digitalRead(13); // Placeholder for LM35-like sensor
    String payload = String(proximity, 2);
    mqttClient.publish(SENSOR_TOPIC, payload.c_str());
    Serial.print("Published proximity: ");
    Serial.println(payload);
}

void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);  // LED pin
    servo_init();
    digitalWrite(2, LOW);  // Start off
    setup_wifi();
    mqttClient.setServer(broker_ip, broker_port);
    mqttClient.setCallback(mqttCallback);
    ledcAttach(SOL_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(SOL_PIN, 0);   // MOSFET OFF initially
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}



void loop() {
    ArduinoOTA.handle();
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 0.8) {
        lastMsg = millis();
        publishSensorData();
    }
    Serial.println(mos_status);
    Serial.print("motor1 speed is:");
    Serial.println(m1);
    Serial.println(m2);
    Serial.println(m3);
}
