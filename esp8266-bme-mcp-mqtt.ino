#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <Wire.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// MQTT Library
#include <PubSubClient.h>

// Adafruit sensor libraries
#include <Adafruit_MCP9808.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
bool hasMCP = false;

// Create BME280 sensor object. (https://www.adafruit.com/product/2652)
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)
bool hasBME = false;

// Default configuration page values for MQTT
// Define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char mqtt_topic[40];
char topic[40];

// Convenience variables to clear settings.
bool resetFS = false;
bool resetWifi = false;

// Create MQTT client object.
WiFiClient espClient;
PubSubClient client(espClient); 

String clientName = "HAB-" + String(ESP.getChipId());

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect((char*) clientName.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  // Check for available sensors. If none are found end the program.
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example
    if (tempsensor.begin()) {
      Serial.println("Found MCP9808!");
      hasMCP = true;
    }
    if (bme.begin()) {
      Serial.println("Found BME280 sensor!");
      hasBME = true;
    }

  //clean FS, for testing
  if (resetFS) {
    SPIFFS.format();
  }

  //read configuration from FS json
  Serial.println("mounting FS...");
  // Borrowed heavily from the WifiManager library.
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          // Custom parameters
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Topic", mqtt_topic, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset settings - for testing
  if (resetWifi) {
    wifiManager.resetSettings();
  }
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect((char*) clientName.c_str(), "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_topic"] = mqtt_topic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, atoi(mqtt_port));

  // Start main program. This is in setup() to allow for deep sleep.
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  char temp_c_str[10];
  char pressure_hpa_str[6];
  char humidity_str[4];
  strcpy(topic, mqtt_topic);

    if (hasMCP) {
      Serial.println("Using MCP9808 Digital temperature sensor.");
      tempsensor.shutdown_wake(0);
      delay(250);
  
      // Read sensor data
      // Get Temperature
      float temp_c;
      temp_c = tempsensor.readTempC();
      dtostrf(temp_c, 3, 2, temp_c_str);
      strcat(topic, "/temperature");
      Serial.print(topic);
      Serial.print(": ");
      Serial.println(temp_c_str);
      client.publish(topic, temp_c_str, true);
    }
    if (hasBME) {
      Serial.println("Using BME280 Bosch Digital temperature/pressure/humidity sensor.");
      // Read sensor data
      // Get Temperature
      float temp_c;
      temp_c = bme.readTemperature();
      dtostrf(temp_c, 3, 2, temp_c_str);
      strcat(topic, "/temperature");
      client.publish(topic, temp_c_str, true);
      strcpy(topic, mqtt_topic);
      
      // Get Pressure
      float pressure_hpa;
      pressure_hpa = bme.readPressure() / 100.0F;
      dtostrf(pressure_hpa, 4, 2, pressure_hpa_str);
      strcat(topic, "/pressure");
      client.publish(topic, pressure_hpa_str, true);
      strcpy(topic, mqtt_topic);

      // Get Humidity
      float rel_humidity;
      rel_humidity = bme.readHumidity();
      dtostrf(rel_humidity, 3, 0, humidity_str);
      strcat(topic, "/humidity");
      client.publish(topic, humidity_str, true);
    }
    Serial.println("Disconnect.");
    client.disconnect();
    delay(250);

  Serial.println("Sleep for 5 minutes");
  ESP.deepSleep(300e6); // in microseconds
}

void loop() {
  
}
