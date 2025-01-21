#include "VOneMqttClient.h"
#include "DHT.h"
#define DHTTYPE DHT11

const int lightThreshold = 50;
int MinLightValue = 200;
int MaxLightValue = 3200;
int MinOutput = 0;
int MaxOutput = 100;
float LightLevel = 0;

boolean inOverride = false;

//define device id
const char* LightMeter = "9d55f4ea-2627-45db-a1bc-2eaffd1d7acf";
const char* MQ2sensor = "dc518777-708d-42cf-8e65-2b004ac00cb9";
const char* dht11Sensor = "34b8e0a0-6f59-40af-9266-7de944d45787";
const char* relayLight = "40b320c4-c7a2-48fc-accd-92aff5cdc7c3";
const char* overrideMode = "cec8ba0c-c793-4959-ac74-6edf36433132";
const char* pirSensor = "87ca7c10-c265-456a-930c-77a1dd1e6553";

//Used Pins
const int lightPin = 36;
const int MQ2pin = 34;
const int dht11Pin = 25;
const int relayPin = 32;
const int overridePin = 27;
const int pirPin = 23;
const int motionPin = 10;

DHT dht(dht11Pin, DHTTYPE);

//input sensor

//Create an instance of VOneMqttClient
VOneMqttClient voneClient;

const int telemetryInterval = 5000;
const int updateInterval = 2000;
const int pirSensorDelay = 15000;

//last message time
unsigned long lastMsgTime = 0;
unsigned long lastUpdateTime = 0;

unsigned long lastTrigger = 0;
boolean PIRvalue = false;

void IRAM_ATTR detectsMovement() {
  PIRvalue = true;
  lastTrigger = millis();
  Serial.println("Motion detected!");
  digitalWrite(motionPin, HIGH);

  if (!inOverride && LightLevel < 50) {
    digitalWrite(relayPin, true);
  }
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  //actuatorCommand format {"servo":90}
  Serial.print("Main received callback : ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";

  JSONVar commandObject = JSON.parse(actuatorCommand);
  JSONVar keys = commandObject.keys();

  if (String(actuatorDeviceId) == overrideMode) {
    String key = "";
    boolean commandValue = false;
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = (bool)commandObject[keys[i]];
      Serial.print("Key : ");
      Serial.println(key.c_str());
      Serial.print("value : ");
      Serial.println(commandValue);
    }

    if (commandValue == true) {
      inOverride = true;
      digitalWrite(overridePin, HIGH);
    } else {
      inOverride = false;
      digitalWrite(overridePin, LOW);
    }

    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
    //Sample publish actuator fail status
    //errorMsg = "LED unable to light up.";
    //voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);
  }


  if (String(actuatorDeviceId) == relayLight) {
    if (inOverride) {
      String key = "";
      boolean commandValue = false;
      for (int i = 0; i < keys.length(); i++) {
        key = (const char*)keys[i];
        commandValue = (bool)commandObject[keys[i]];
        Serial.print("Key : ");
        Serial.println(key.c_str());
        Serial.print("value : ");
        Serial.println(commandValue);
      }

      if (commandValue == true) {
        Serial.println("Relay ON");
        digitalWrite(relayPin, true);
      } else {
        Serial.println("Relay OFF");
        digitalWrite(relayPin, false);
      }

      voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
    } else {
      // errorMsg = "Override mode is not on.";
      // voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);
      voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
    }
  }
}

void setup() {
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  dht.begin();
  pinMode(relayPin, OUTPUT);
  pinMode(overridePin, OUTPUT);
  pinMode(motionPin, OUTPUT);
  pinMode(pirPin, INPUT);
  inOverride = false;

  attachInterrupt(digitalPinToInterrupt(pirPin), detectsMovement, RISING);
}

void loop() {

  if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "Sensor Fail";
    voneClient.publishDeviceStatusEvent(LightMeter, true);
    voneClient.publishDeviceStatusEvent(MQ2sensor, true);
    voneClient.publishDeviceStatusEvent(dht11Sensor, true);
    voneClient.publishDeviceStatusEvent(pirSensor, true);
  }
  voneClient.loop();


  unsigned long cur = millis();
  if (!inOverride && cur - lastUpdateTime > updateInterval) {
    lastUpdateTime = cur;

    int lightValue = analogRead(lightPin);
    LightLevel = map(lightValue, MinLightValue, MaxLightValue, MinOutput, MaxOutput);

    if (PIRvalue && cur - lastTrigger > pirSensorDelay) {
      PIRvalue = false;
      digitalWrite(motionPin, LOW);
      voneClient.publishTelemetryData(pirSensor, "Motion", PIRvalue);
    }

    if (LightLevel < lightThreshold && PIRvalue) {
      digitalWrite(relayPin, true);
    } else {
      digitalWrite(relayPin, false);
    }
  }
  if (cur - lastMsgTime > telemetryInterval) {
    lastMsgTime = cur;

    //Publish telemtry data
    voneClient.publishTelemetryData(LightMeter, "Light", LightLevel);
    voneClient.publishTelemetryData(pirSensor, "Motion", PIRvalue);

    float gasValue = analogRead(MQ2pin);
    voneClient.publishTelemetryData(MQ2sensor, "Gas detector", gasValue);

    float humidity = dht.readHumidity();        // Read humidity
    float temperature = dht.readTemperature();  // Read temperature

    JSONVar payloadObject;
    payloadObject["Humidity"] = humidity;
    payloadObject["Temperature"] = temperature;
    voneClient.publishTelemetryData(dht11Sensor, payloadObject);
  }
}