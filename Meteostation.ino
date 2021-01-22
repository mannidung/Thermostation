
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <InfluxDbClient.h>
#include <uptime.h>
#include "WIFICredentials.h"
#include "InfluxDBCredentials.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#define UNIT_TAG "kitchen"
// Delay between measurements
#define DELAY 60000
#define SPI_ADDRESS 0x76
#define BLINK 0

// ####### WIFI ######
char wifi_ssid[] = WIFI_SSID;
char wifi_pass[] = WIFI_PASS;

// ####### InfluxDB ######

// InfluxDB 2 client instance
// InfluxDBClient client(INFLUXDB2_URL, INFLUXDB2_ORG, INFLUXDB2_BUCKET, INFLUXDB2_TOKEN);

// InfluxDB 1 client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DBNAME);

unsigned long delayTime;

// Initiate sensor and datapoint
Adafruit_BME280 bme; // I2C
Point sensor(UNIT_TAG);

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println(F("METEOSTATION"));
    Serial.print(F("UNIT NAME: "));
    Serial.println(F(UNIT_TAG));

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(1000);                       // wait for a second 
        while (1) delay(10);
    }

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

    // Connect WiFi
    Serial.println("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID, WIFI_PASS);
    while (wifiMulti.run() != WL_CONNECTED) {
      Serial.print(".");
      delay(500); 
    }
    Serial.println();

      // Check server connection
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());
      // Add tags
      sensor.addTag("device", DEVICE);
      sensor.addTag("unit", UNIT_TAG); 
    } else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
    }

    blink();
}


void loop() { 
    postValues();
    delay(DELAY);
}

void postValues() {
    // Store measured value into point
    sensor.clearFields();
    bme.takeForcedMeasurement();
    // Report RSSI of currently connected network
    sensor.addField("rssi", WiFi.RSSI());
    sensor.addField("uptime", 1);
    sensor.addField("temperature", bme.readTemperature());
    sensor.addField("pressure", bme.readPressure() / 100.0F);
    sensor.addField("humidity", bme.readHumidity());
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
    if (BLINK) {
      blink();
    }
    
}

void blink() {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
    delay(100);
}
