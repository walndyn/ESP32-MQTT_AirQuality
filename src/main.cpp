// ---include section---
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_SGP30_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <EspMQTTClient.h>
#include <numeric>
#include "credentials.h"

// ---define section---
#define debug false         // Enable/Disable Debug functionality (serial output)
#define DHTPIN 26           // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22       // DHT22 or DHT11
#define DHTreadouttime 2    // DHT Sensor readout time in seconds
#define SGP30readouttime 2  // SGP30 Sensor readout time in seconds
#define publish_interval 60 // Publish readings to MQTT time in seconds
#define tempAdjust 1.5      // Adjusts the DHT sensors temperature values negativly => 1 = temp - 1C°
#define humidityAdjust 3    // Adjusts the DHT sensors humidity values negativly => 1 = humidity - 1%
#define doorMargin 60       // Hysterisis margine for door status algorithm
#define doorTreshold 700    // Door status treshold (0-4096 ADC)
#define lightbarrierPin 35  // ADC Pin connected to the light barrier
#define triggerPin 33       // Pin connected to PIR
#define PirDelay 5000       // Default PIR delay time before next trigger

// ---create variables---
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
volatile unsigned long previousMillis4 = 0;
const long DHTreadout = DHTreadouttime * 1000;
const long SGP30readout = SGP30readouttime * 1000;
const long PublishInterval = publish_interval * 1000;
float humidity = 50;
float temp = 20;
String h;
String t;
String co2;
String tvoc;
String door = "closed"; // true/1 equals open
int doorValue;
volatile int triggerCnt = 1;
volatile bool intTrigger = false;
bool timer;
std::vector<int> co2Vector;
std::vector<int> tvocVector;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor
SGP30 sgp30;              // create an object of the SGP30 class

EspMQTTClient client(
    SSID,         // Wifi SSID
    WifiPassword, // Wifi Password
    brokerIP,     // MQTT Broker server ip
    clientName,   // Client name that uniquely identifys your device
    //"MQTTUsername",   // Can be omitted if not needed
    //"MQTTPassword",   // Can be omitted if not needed
    1883 // The MQTT port, default to 1883. this line can be omitted

);

// ---PIR interrupt function---
void IRAM_ATTR ISR()
{
  portENTER_CRITICAL_ISR(&mux);
  if (triggerCnt == 1)
  {
    intTrigger = true;
    previousMillis4 = millis();
  }

  triggerCnt++;
  portEXIT_CRITICAL_ISR(&mux);
}

// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  if (debug == true)
    (Serial.println("going online..."));
  // ---Subscribe to MQTT topics---
  client.subscribe("bedroom/espbed/CO2", [](const String &payload) {});
  client.subscribe("bedroom/espbed/TVOC", [](const String &payload) {});
  client.subscribe("bedroom/espbed/temperature", [](const String &payload) {});
  client.subscribe("bedroom/espbed/humidity", [](const String &payload) {});
  client.subscribe("bedroom/espbed/baselineCO2", [](const String &payload) {});
  client.subscribe("bedroom/espbed/baselineTVOC", [](const String &payload) {});
  client.subscribe("bedroom/espbed/battery", [](const String &payload) {});
  client.subscribe("bedroom/espbed/door", [](const String &payload) {});

  /* Subscribe to "mytopic/wildcardtest/#" and display received message to Serial
  client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
    if(debug == true)(Serial.println("(From wildcard) topic: " + topic + ", payload: " + payload));
  });*/

  client.publish("bedroom/espbed", "ESP32-Bed/Air is online"); // Publish a message, you can activate the retain flag by setting the third parameter to true
  if (debug == true)
    (Serial.println("ESP32-Bed is online"));
}

// ---Convert relative humidity to absolute for SGP30 humidity correction---
double RHtoAbsolute(float relHumidity, float tempC)
{
  double eSat = 6.11 * pow(10.0, (7.5 * tempC / (237.7 + tempC)));
  double vaporPressure = (relHumidity * eSat) / 100;                         // millibars
  double absHumidity = 1000 * vaporPressure * 100 / ((tempC + 273) * 461.5); // Ideal gas law with unit conversions
  return absHumidity;
}

// ---Convert double to uint16_t---
uint16_t doubleToFixedPoint(double number)
{
  int power = 1 << 8;
  double number2 = number * power;
  uint16_t value = floor(number2 + 0.5);
  return value;
}

// ---check the door status an publish if changed + hysterisis control---
void doorStatus()
{
  String lastDoor = door;
  doorValue = analogRead(lightbarrierPin);

  if (doorValue <= (doorTreshold - doorMargin))
  {
    door = "opened";
  }
  else if (doorValue >= (doorTreshold + doorMargin))
  {
    door = "closed";
  }

  if (lastDoor != door)
  {
    if (debug == true)
      (Serial.println(""));
    client.publish("bedroom/espbed/door", String(door));
    if (debug == true)
    {
      Serial.print("door: ");
      Serial.println(door);
    }
  }
}

// ---check if PIR triggered interrupt and publish---
void PIR()
{
  if (intTrigger == true)
  {
    client.publish("bedroom/espbed/door", "triggered");
    if (debug == true)
      (Serial.println("triggered"));
    portENTER_CRITICAL(&mux);
    intTrigger = false;
    portEXIT_CRITICAL(&mux);
  }
}

void setup()
{
  if (debug == true)
    (Serial.begin(115200));
  Wire.begin();

  // ---functionalities of EspMQTTClient---
  // client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(username, webPassword); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA();                                 // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  // client.enableLastWillMessage("bedroom/espbed", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

  // ---initializes DHT sensor----
  dht.begin();

  // ---initialize SGP30 sensor----
  if (debug == true)
  {
    Serial.println("");
    Serial.println("DHT initialized...");

    if (sgp30.begin() == false)
      Serial.println("No SGP30 Detected. Check connections.");
    else
      Serial.println("SGP30 initialized...");
  }
  else
    (sgp30.begin());

  delay(15000);

  sgp30.initAirQuality(); // measureAirQuality should be called in one second increments after a call to initAirQuality

  pinMode(lightbarrierPin, INPUT_PULLDOWN);
  pinMode(triggerPin, INPUT_PULLDOWN);
  attachInterrupt(33, ISR, RISING);

  if (debug == true)
    (Serial.println("setting up done!"));
}

void loop()
{
  client.loop();

  // ---check the door status---
  doorStatus();

  // ---check PIR status---
  PIR();

  // ---timer for DHT readout---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= DHTreadout)
  {
    previousMillis = currentMillis;

    humidity = dht.readHumidity() - humidityAdjust;
    temp = dht.readTemperature() - tempAdjust;

    h = String(humidity);
    t = String(temp);

    // Check if any reads failed and exit early (to try again).
    if (isnan(dht.readHumidity()) || isnan(dht.readTemperature()))
    {
      if (debug == true)
        (Serial.println(F("Failed to read from DHT sensor!")));
      return;
    }
  }

  // ---timer for SGP30 readout---
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= SGP30readout)
  {
    previousMillis2 = currentMillis2;

    double absHumidity = RHtoAbsolute(humidity, temp);       // Convert relative humidity to absolute humidity
    uint16_t sensHumidity = doubleToFixedPoint(absHumidity); // Convert the double type humidity to a fixed point 8.8bit number

    sgp30.setHumidity(sensHumidity); // send humidity correction

    sgp30.measureAirQuality();

    co2Vector.push_back(sgp30.CO2);
    tvocVector.push_back(sgp30.TVOC);
  }

  // ---timer for Publishing to MQTT---
  unsigned long currentMillis3 = millis();
  if (currentMillis3 - previousMillis3 >= PublishInterval)
  {
    previousMillis3 = currentMillis3;

    co2 = String(std::accumulate(co2Vector.begin(), co2Vector.end(), 0) / co2Vector.size());
    tvoc = String(std::accumulate(tvocVector.begin(), tvocVector.end(), 0) / tvocVector.size());

    sgp30.getBaseline();

    client.publish("bedroom/espbed/temperature", t);
    client.publish("bedroom/espbed/humidity", h);
    client.publish("bedroom/espbed/CO2", co2);
    client.publish("bedroom/espbed/TVOC", tvoc);
    client.publish("bedroom/espbed/BaselineCO2", String(sgp30.baselineCO2));
    client.publish("bedroom/espbed/BaselineTVOC", String(sgp30.baselineTVOC));
    client.publish("bedroom/espbed/battery", String(analogRead(34) / (4096 / 3.3) * 2));

    if (debug == true)
    {
      Serial.println("");
      Serial.print("temperature: ");
      Serial.println(t);
      Serial.print("humidity: ");
      Serial.println(h);
      Serial.print("CO2: ");
      Serial.println(co2);
      Serial.print("TVOC: ");
      Serial.println(tvoc);
      Serial.print("BaselineCO2: ");
      Serial.println(sgp30.baselineCO2);
      Serial.print("BaselineTVOC: ");
      Serial.println(sgp30.baselineTVOC);
      Serial.print("battery: ");
      Serial.println(analogRead(34) / (4096 / 3.3) * 2);
    }

    co2Vector.clear();
    tvocVector.clear();
  }

  // ---timer for resetting PIR status---
  unsigned long currentMillis4 = millis();
  if ((currentMillis4 - previousMillis4) > PirDelay)
  {
    portENTER_CRITICAL(&mux);
    triggerCnt = 1;
    portEXIT_CRITICAL(&mux);
  }
}