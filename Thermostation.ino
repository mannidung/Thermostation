#include <Adafruit_BME280.h>
#include <InfluxDbClient.h>
#include <uptime.h>
#include "WIFICredentials.h"
#include "InfluxDBCredentials.h"

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#define SPI_ADDRESS 0x76

// ####### CONFIG #######
// The user may/should change these settings as he wishes
const char* UNIT_TAG = "bedroom"; // The name of the unit that will show up in InfluxDB
const int DELAY = 60e6; // Delay between measurements in microseconds

// ####### InfluxDB ######

// InfluxDB 2 client instance
// InfluxDBClient client(INFLUXDB2_URL, INFLUXDB2_ORG, INFLUXDB2_BUCKET, INFLUXDB2_TOKEN);

// InfluxDB 1 client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DBNAME);

// Initiate sensor and datapoint
Adafruit_BME280 bme; // Use I2C as interface
Point sensor(UNIT_TAG);

void connectSensor() {
    unsigned status;

    // Connect to sensor
    status = bme.begin(SPI_ADDRESS);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        delay(1000);                       // wait for a second 
        while (1) delay(10);
    }
    Serial.println();
    Serial.println("Successfully connected to sensor, setting forced mode...");
    Serial.println();
    Serial.println();

    // Set forced mode to lower power consumption
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    Serial.println();
    Serial.println("Forced mode set, sensor ready...");
    Serial.println();
    Serial.println();
}

void connectWifi() {
    // Connect WiFi
    Serial.println("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID, WIFI_PASS);
    while (wifiMulti.run() != WL_CONNECTED) {
      Serial.print(".");
      delay(1000); 
    }
    
    Serial.println();
    Serial.println("Connected to Wifi");
    Serial.println();
}

void setMeasurementTags() {
  sensor.addTag("device", DEVICE);
  sensor.addTag("unit", UNIT_TAG);
}

void readSensor() {
    // Store measured value into point
    sensor.clearFields();
    bme.takeForcedMeasurement();
    // Report RSSI of currently connected network
    sensor.addField("rssi", WiFi.RSSI());
    sensor.addField("temperature", bme.readTemperature());
    sensor.addField("pressure", bme.readPressure() / 100.0F);
    sensor.addField("humidity", bme.readHumidity());
    
}

void submitMeasurement() {
    // Print what are we exactly writing
    Serial.print("Writing: ");
    Serial.println(client.pointToLineProtocol(sensor));
    // If no Wifi signal, try to reconnect it
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Wifi connection lost");
    }
    // Write point
    if (!client.writePoint(sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
}

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    
    Serial.println();
    Serial.println();
    Serial.println(F("METEOSTATION"));
    Serial.print(F("UNIT NAME: "));
    Serial.println(UNIT_TAG);

    // Connect the BME280 sensor
    connectSensor();
    // Connect to the Wifi
    connectWifi();
    // Set the tags of the measurement
    setMeasurementTags();

    // Do the measurement
    readSensor();
    submitMeasurement();


    Serial.println();

    Serial.println("Sleeping until next cycle...");
    ESP.deepSleep(DELAY); // Going into deep sleep
}

void loop() { 
}
