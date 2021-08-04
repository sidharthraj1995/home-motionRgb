/*
ESP8266 based monitoring system
Board: ESP8266 12E
Sensors: HW-478
         RGB control
Motion triggers light and RGB control
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <time.h>

// Replace the next variables with your SSID/Password combination
char* ssid = "YOUR_SSID_HERE";
const char* password = "YOUR_WIFI_PASSWORD";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "MQTT_SERVER_IP";
//const char* mqtt_server = "localhost";

#define timeSeconds 10

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long localadd;
char msg[50];
int value = 0;

// Set GPIOs for LED and PIR Motion Sensor
const int lightPin = 3;    //
const int motionPin = 2;  //D4
const int irPin = 16;   //D0

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

//Topics
const char* clientTopic = "home/stairs/client";
const char* motionTopic = "home/stairs/motion";
const char* lightTopic = "home/stairs/bulb";
const char* rgbTopic = "home/stairs/rgbColor";

//Topic messages
char* motionOff = "0";
char* glightOff = "0";
char* motionOn = "1";
char* glightOn = "1";
char* rgbOff = "0";
char* rgbOn = "1";


//Current states
bool motionState = false;
bool rgbState = false;
bool lightState = false;
bool clientState = false;


// Checks if motion was detected, sets LED HIGH and starts a timer
IRAM_ATTR void detectsMovement() {
  startTimer = true;
  lastTrigger = millis();
  motionState = 1;
  Serial.print("[TRIGGER]MOTION DETECTED at:");
  Serial.println(millis());
  client.publish("home/stairs/motion", motionOn);
  digitalWrite(lightPin, HIGH);
  lightState = 1;
  Serial.print("[TRIGGER]Turning guide lights: ON");
  client.publish("home/stairs/guideLight", glightOn);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.println();
  Serial.print("[MESSAGE]***Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Length: ");
  Serial.print(length);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.print("***");
  Serial.println();



/////////Controlling Stairs Bulb//////////////////////////////////////
  if (String(topic) == lightTopic) {
    Serial.println("[ACK]Turning Stair bulb: ");
    if(messageTemp == "1"){
      Serial.println("ON");
      lightState = 1;
      digitalWrite(lightPin, HIGH);
    }
    else if(messageTemp == "0"){
      Serial.println("OFF");
      lightState = 0;
      digitalWrite(lightPin, LOW);
    }
  }
  
/////////Controlling RGB light//////////////////////////////////////
  if (String(topic) == rgbTopic){
    
    Serial.print("[ACK]RGB received: ");
    Serial.println(messageTemp);

    long number = (long) strtol( &messageTemp[0], NULL, 16);
    //Split them up into r, g, b values
    long r = ((number >> 16) & 0xFF);
    long g = ((number >> 8) & 0xFF);
    long b = (number & 0xFF);

    Serial.println("[FORCE]RGB LED set to: ");
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.print(b);
    
  }
}



void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionPin, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionPin), detectsMovement, RISING);

  // Configure IOs
  pinMode(lightPin, OUTPUT);
  pinMode(irPin, OUTPUT);

  //Set States to LOW
  digitalWrite(lightPin, LOW);
  digitalWrite(irPin, LOW);

  setup_wifi();
  client.setServer(mqtt_server, 1024);
  client.setCallback(callback);

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("###Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("test_ELSA8266")) {
      Serial.println("+++ELSA8266+++ connected");
      // Subscribe
      client.subscribe("home/#");
      clientState = 1;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      clientState = 0;
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Current time
  now = millis();

  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("[FORCE]Motion stopped...");
    motionState = 0;
    client.publish(motionTopic , motionOff);
    digitalWrite(lightPin, LOW);
    Serial.println("[FORCE]Turning guide lights off.");
    lightState = 0;
    client.publish(motionTopic , glightOff);
    startTimer = false;
  } 
}