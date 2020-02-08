#include <QList.h>

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* PINS */
#define PIN_WIFI_LED  D7
#define PIN_MQTT_LED  D8
#define PIN_QUEUE_LED D6
#define PIN_TX_LED    D5
#define PIN_433_RX    D2
#define PIN_433_TX    D1

/* WIFI */
#define WIFI_SSID  "WIFI_SSID"
#define WIFI_PASS  "WIFI_PASS"
#define WIFI_HOST  "shutter.local"

/* MQTT */
#define MQTT_HOST         "MQTT_IP_OR_HOST"
#define MQTT_USER         "MQTT_USER"
#define MQTT_PASS         "MQTT_PASS"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "mqtt/shutter" // This controller's MQTT clientID
#define MQTT_WILL_TOPIC   "home/shutter/contrller/availability" // This controller's availability topic
#define MQTT_WILL_QOS     1
#define MQTT_WILL_RETAIN  0
#define MQTT_WILL_MESSAGE "offline"

#define MQTT_HA_AVAILABILITY  "home/ha/availability" // Home assistant/MQTT broker availability topic

#define SHUTTER_MQTT_TOPIC_SET          "/set"
#define SHUTTER_MQTT_TOPIC_STATE        "/state"
#define SHUTTER_MQTT_TOPIC_AVAILABILITY "/availability"
#define SHUTTER_MQTT_TOPIC_CONFIG       "/config"

/* 433 REMOTE CONTROL PROTOCOL SETTINGS */
#define PULSE_LENGTH      380 // Length of low or high pulses
#define PULSE_BIT_LENGTH  3   // Length of one bit (PULSE_LENGTH * PULSE_BIT_LENGTH is the lenght of a bit)
#define PULSE_SYNC_HIGH   12  // Number of pulses in HIGH state to for the sync
#define PULSE_SYNC_LOW    3   // Number of pulses in LOW state to for the sync
#define PULSE_1_HIGH      2   // Number of pulses in HIGH for the bit vaue 1
#define PULSE_1_LOW       1   // Number of pulses in LOW for the bit vaue 1
#define PULSE_0_HIGH      1   // Number of pulses in HIGH for the bit vaue 0
#define PULSE_0_LOW       2   // Number of pulses in LOW for the bit vaue 0

/* Structure to store status related data */
typedef struct
{
  /* The timestamp when the status was checked the last time */
  unsigned long lastStatusCheck;
  /* Number of milliseconds between status checks */
  unsigned long statusCheckInterval;
} StatusStruct;

WiFiClient wifiClient;
StatusStruct wifiStatus = {0, 500};

PubSubClient mqttClient(wifiClient);
StatusStruct mqttStatus = {0, 500};
StatusStruct mqttAvailability = {0, 5000};

typedef struct
{
  char mqttState[8];
  char mqttSet[6];
  byte rf[2];
  byte state;
} ShutterState;

const byte STATE_UNDEFINED  = 0;
const byte STATE_OPEN       = 1;
const byte STATE_CLOSED     = 2;
const byte STATE_STOPPED    = 3;
const byte STATE_LEARN      = 4;

const byte shutterStateCount = 5;
const ShutterState shutterStates[shutterStateCount] = {
    { "",         "",       { 0b00000000, 0b00000000 }  } /*shutterStates[STATE_UNDEFINED]  = */
  , { "open",     "OPEN",   { 0b00010001, 0b00011110 }  } /*shutterStates[STATE_OPEN]       = */
  , { "closed",   "CLOSE",  { 0b00110011, 0b00111100 }  } /*shutterStates[STATE_CLOSED]     = */
  , { "stopped",  "STOP",   { 0b01010101, 0b00000000 }  } /*shutterStates[STATE_STOPPED]    = */
  , { "learn",    "LEARN",  { 0b11001100, 0b00000000 }  } /*shutterStates[STATE_LEARN]      = */
};

typedef struct
{
  char mqttTopic[64];
  long remoteId;
  byte channel;
} Shutter;

const byte shutterCount = 2;
const byte UNKNOWN_SHUTTER = 255;
Shutter shutters[shutterCount] = {
    {"home/shutter/test/1", 123456789,  1 }
  , {"home/shutter/test/2", 123456789,  2 }
};

typedef struct
{
  byte shutter;
  byte state;
} Command;

QList<Command> mqttCommandQueue;
StatusStruct mqttQueueStatus = {0, 500};

/**
 * 
 */
void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println(" >> Booting...");
  
  
  pinMode(PIN_WIFI_LED, OUTPUT);
  pinMode(PIN_MQTT_LED, OUTPUT);
  pinMode(PIN_QUEUE_LED, OUTPUT);
  pinMode(PIN_TX_LED, OUTPUT);
  
  pinMode(PIN_433_TX, OUTPUT);
  pinMode(PIN_433_RX, INPUT);

  testPins();
  
  digitalWrite(PIN_WIFI_LED, LOW);
  digitalWrite(PIN_MQTT_LED, LOW);
  digitalWrite(PIN_QUEUE_LED, LOW);
  digitalWrite(PIN_433_TX, LOW);

  wifiSetup();  
  mqttSetup();

  Serial.println(" >> Ready...");
}

void testPins()
{
  digitalWrite(PIN_WIFI_LED, HIGH);
  digitalWrite(PIN_MQTT_LED, HIGH);
  digitalWrite(PIN_QUEUE_LED, HIGH);
  digitalWrite(PIN_433_TX, HIGH);
  digitalWrite(PIN_TX_LED, HIGH);

  delay(1000);

  digitalWrite(PIN_WIFI_LED, LOW);
  digitalWrite(PIN_MQTT_LED, LOW);
  digitalWrite(PIN_QUEUE_LED, LOW);
  digitalWrite(PIN_433_TX, LOW);
  digitalWrite(PIN_TX_LED, LOW);
}

void loop() {
  wifiCheckStatus();
  if (!mqttClient.loop())
  {
    mqttCheckStatus();
  }

  processQueue();
}

byte findStateByMqttSet(char * mqttSet)
{
  for (byte i=1; i<shutterStateCount; i++)
  {
    if (strcmp(mqttSet, shutterStates[i].mqttSet) == 0)
    {
      return i;
    }
  }

  return STATE_UNDEFINED;
}

byte findStateByMqttState(char * mqttState)
{
  for (byte i=1; i<shutterStateCount; i++)
  {
    if (strcmp(mqttState, shutterStates[i].mqttState) == 0)
    {
      return i;
    }
  }

  return STATE_UNDEFINED;
}

byte findShutterByMqttTopic(char * mqttTopic)
{
  for (byte i=1; i<shutterCount; i++)
  {
    if (strncmp(mqttTopic, shutters[i].mqttTopic, strlen(shutters[i].mqttTopic)) == 0)
    {
      return i;
    }
  }

  return UNKNOWN_SHUTTER;
}

void processQueue()
{
  processMqttCommandQueue();
}

void processMqttCommandQueue()
{
  unsigned long now = millis();
  if (mqttCommandQueue.size() > 0)
  {
    if ((now - mqttQueueStatus.lastStatusCheck) >  mqttQueueStatus.statusCheckInterval)
    {
      Command command;
    
      command = mqttCommandQueue.back();
      processMqttCommand(command);
      mqttCommandQueue.pop_back();

      mqttQueueStatus.lastStatusCheck = now;
    }
  }
  mqttQueueStatusToLed();
}

void mqttQueueStatusToLed()
{
  digitalWrite(PIN_QUEUE_LED, (mqttCommandQueue.size() > 0 ? HIGH : LOW ));
}

void processMqttCommand(Command command)
{
  rfSendAction(shutters[command.shutter].remoteId, shutters[command.shutter].channel, command.state);
  mqttPublishState(command.shutter, command.state);
}

/**
 * Configures the wifi connection
 */
void wifiSetup()
{
  unsigned long now = millis();
  unsigned long lastCheck = millis();
  unsigned long timeout = millis() + 30000; // 30s timeout
  byte st = WiFi.status();
  
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while(true);  // don't continue
  }

  if (WiFi.SSID() != WIFI_SSID || WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.config(ip, dns, gw, mask);
    WiFi.hostname(WIFI_HOST);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.println("Connecting to WiFi");
    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      Serial.print("Connected to ");
      Serial.print(WIFI_SSID);
      Serial.print(" | IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(" | Signal Strength: ");
      Serial.print(WiFi.RSSI());
      Serial.print("dBm");
      Serial.println("");
    }
  }
  
  wifiCheckStatus();
}

/**
 * Checks if wifi is still connected
 */
void wifiCheckStatus()
{
  unsigned long now = millis();
  if ((now - wifiStatus.lastStatusCheck) >  wifiStatus.statusCheckInterval)
  {
    wifiStatus.lastStatusCheck = now;
    if (WiFi.status() != WL_CONNECTED)
    {
      ESP.reset();
    }
    wifiStatusToLed();
  }
}

/**
 * 
 */
void wifiStatusToLed()
{
  digitalWrite(PIN_WIFI_LED, ((WiFi.status() == WL_CONNECTED) ? HIGH : LOW ));
}

/**
 * MQTT
 */
void mqttSetup()
{
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

/**
 * 
 */
void mqttConnect()
{
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_WILL_TOPIC, MQTT_WILL_QOS, MQTT_WILL_RETAIN, MQTT_WILL_MESSAGE))
  {
    mqttSubscribe();
    mqttPublishAvailablity(true);
  }
}

void mqttSubscribe()
{
  mqttClient.subscribe(MQTT_HA_AVAILABILITY, 1);
  char topic[64];
  for (byte i = 0; i < shutterCount; i++)
  {
    sprintf(topic, "%s%s", shutters[i].mqttTopic, SHUTTER_MQTT_TOPIC_SET);
    Serial.print("Subscribing to topic [");
    Serial.print(topic);
    Serial.print("]");
    Serial.println("");
    mqttClient.subscribe(topic, 1);
  }
}

void mqttPublishState(byte shutter, byte state)
{
  char topic[64];
  sprintf(topic, "%s%s", shutters[shutter].mqttTopic, SHUTTER_MQTT_TOPIC_STATE);
  mqttClient.publish(topic, shutterStates[state].mqttState);

  if (shutters[shutter].channel == 0) {
    for (byte i=0; i<shutterCount; i++)
    {
      if (shutters[i].remoteId == shutters[shutter].remoteId && shutters[i].channel > 0)
      {
        mqttPublishState(i, state);
      }
    }
  }
}

void mqttPublishAvailablity()
{
  return mqttPublishAvailablity(false);
}
void mqttPublishAvailablity(bool force)
{
  unsigned long now = millis();
  mqttClient.publish(MQTT_WILL_TOPIC, "online");
  if (mqttClient.connected())
  {
    if (force || ((now - mqttAvailability.lastStatusCheck) >  mqttAvailability.statusCheckInterval))
    {
      mqttAvailability.lastStatusCheck = now;
      char topic[64];
      for (byte i = 0; i < shutterCount; i++)
      {
        sprintf(topic, "%s%s", shutters[i].mqttTopic, SHUTTER_MQTT_TOPIC_AVAILABILITY);
        mqttClient.publish(topic, "online");
      }
      mqttStatusToLed();
    }
  }
}

/**
 * 
 */
void mqttCheckStatus()
{
  mqttCheckStatus(false);
}
void mqttCheckStatus(bool force)
{
  unsigned long now = millis();
  if (force || ((now - mqttStatus.lastStatusCheck) >  mqttStatus.statusCheckInterval))
  {
    mqttStatus.lastStatusCheck = now;
    if (WiFi.status() == WL_CONNECTED)
    {
      if (!mqttClient.connected())
      {
        mqttConnect();
      }
    }
  }
}

/**
 * 
 */
void mqttStatusToLed()
{
  digitalWrite(PIN_MQTT_LED, (mqttClient.connected() ? HIGH : LOW ));
}

/**
 * 
 */
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = NULL;

  Serial.print("MQTT [ ");
  Serial.print(topic);
  Serial.print(" ] : [ ");
  Serial.print(message);
  Serial.print(" ]");

  // Home Assistant availability topic received
  if (strcmp(topic, MQTT_HA_AVAILABILITY) == 0)
  {
    if (strcmp(message, "online"))
    {
      mqttPublishAvailablity();
    }
  }
  else
  {
    Command command;
    command.state = findStateByMqttSet(message);
    
    if (command.state != STATE_UNDEFINED)
    {
      command.shutter = findShutterByMqttTopic(topic);
      if (command.shutter != UNKNOWN_SHUTTER)
      {
        mqttCommandQueue.push_back(command);
        Serial.print(" [ QUEUED: ");
        Serial.print(command.shutter);
        Serial.print(".");
        Serial.print(command.state);
        Serial.print(" ] ");
      }
    }
  }
  Serial.println("");
}

void rfSendAction(long remoteId, byte channel, byte state)
{
  digitalWrite(PIN_TX_LED, HIGH);
  for (byte i=0; i<2; i++)
  {
    if (shutterStates[state].rf[i] != 0)
    {
      rfSendCommand(remoteId, channel, shutterStates[state].rf[i]);
    }
  }
  digitalWrite(PIN_TX_LED, LOW);
}

void rfSendCommand(long remoteId, byte channel, byte rfCommand)
{
  for (byte i=0; i<7; i++)
  {
    delayMicroseconds(PULSE_LENGTH);
    rfTransmitCommand(remoteId, channel, rfCommand);
    delayMicroseconds(PULSE_LENGTH);
  }
}

void rfTransmitCommand(long remoteId, byte channel, byte rfCommand)
{
  rfTransmitSync();  
  for (byte i=28; i>0; i--)
  {
    rfTransmitBit(bitRead(remoteId, i-1));
  }

  for (byte i=4; i>0; i--)
  {
    rfTransmitBit(bitRead(channel, i-1));
  }

  for (byte i=8; i>0; i--)
  {
    rfTransmitBit(bitRead(rfCommand, i-1));
  }
}

void rfTransmitSync()
{
  rfTransmit(PULSE_SYNC_HIGH, PULSE_SYNC_LOW);
}

void rfTransmitBit(bool b)
{
  //Serial.print(b);
  if (b)
  {
    rfTransmit(PULSE_1_HIGH, PULSE_1_LOW);
  }
  else
  {
    rfTransmit(PULSE_0_HIGH, PULSE_0_LOW);
  }
}

void rfTransmit(int highTime, int lowTime)
{
  digitalWrite(PIN_433_TX, HIGH);
  delayMicroseconds(PULSE_LENGTH * highTime);
  digitalWrite(PIN_433_TX, LOW);
  delayMicroseconds(PULSE_LENGTH * lowTime);  
}
