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
#include <IRremote.h>

// Replace the next variables with your SSID/Password combination
char* ssid = "YOUR_SSID_HERE";
const char* password = "YOUR_WIFI_PASSWORD";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "MQTT_SERVER_IP";
//const char* mqtt_server = "localhost";

#define timeSeconds 10

// Topics
#define clientTopic "home/stairs/client"
#define motionTopic "home/stairs/motion"
#define lightTopic  "home/stairs/bulb"
#define rgbTopic    "home/stairs/rgbColor"
#define irTopic     "home/stairs/irled"

//Name of the module -- client
const char* CLIENTNAME = "stairsRGB";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long localadd;
char msg[50];
int value = 0;

// Set GPIOs for LED and PIR Motion Sensor
const int lightPin = 3;     //
const int motionPin = 2;    //D4
const int irledPin = 12;    //D6

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

//Topics
// const char* clientTopic = "home/stairs/client";
// const char* motionTopic = "home/stairs/motion";
// const char* lightTopic  = "home/stairs/bulb";
// const char* rgbTopic    = "home/stairs/rgbColor";
// const char* irTopic     = "home/stairs/irled";

//Topic messages
char* motionOff = "0";
char* glightOff = "0";
char* motionOn = "1";
char* glightOn = "1";


//Current states
bool motionState = 0;
bool rgbState = 0;
bool lightState = 0;
bool irState = 0;
bool networkState = 0;
bool clientState = 0;

//Create an IR send object
IRsend irsend;

// Checks if motion was detected, sets LED HIGH and starts a timer
ICACHE_RAM_ATTR void detectsMovement() {
  startTimer = true;
  lastTrigger = millis();
  motionState = 1;   //Update motion state
  Serial.print("[TRIGGER]MOTION DETECTED at:");
  Serial.println(millis());
  client.publish(motionTopic, motionOn);
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
    networkState = 0;
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  networkState = 1;
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

  //Command for Light
  if (String(topic) == lightTopic) {
    Serial.println("[ACK]Turning Stair bulb: ");
    if(messageTemp == "1"){
      Serial.println("ON");
      lightState = 1;
      digitalWrite(lightPin, lightState);
    }
    else if(messageTemp == "0"){
      Serial.println("OFF");
      lightState = 0;
      digitalWrite(lightPin, lightState);
    }
  }
  
  //Command for RGB Color
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("###Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CLIENTNAME)) {
      Serial.printf("Client: %s connected!\n", CLIENTNAME);
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


void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionPin, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionPin), detectsMovement, RISING);

  // Configure IOs
  pinMode(lightPin, OUTPUT);
  pinMode(irledPin, OUTPUT);

  //Set States to LOW
  digitalWrite(lightPin, lightState);
  digitalWrite(irledPin, irState);

  setup_wifi();
  client.setServer(mqtt_server, 1024);
  client.setCallback(callback);

  // Initializes IR sender
  //irsend.begin(irledPin); 
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // the address 0x0102 with the command 0x34 is sent 
  IrSender.sendNEC(0x0102, 0x34, true, 0); 

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