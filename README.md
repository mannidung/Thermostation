# THERMOSTATION

Thermostation is a solution for measuring temperature, pressure, and humidity, and sending these measurements to an InfluxDB instance for further analysis.

## How it works
This solution uses an ESP8266 board and a BME280 Temperature/Pressure/Humidity sensor to periodically send information about the temperature, pressure, and humitidy to an InfluxDB instance.

The board will connect to the wifi specified in WIFICredentials.h and send the data to the InfluxDB instance specified in InfluxDBCredentials.h. After the measurement has been done and sent, the unit will go into deep sleep for a specified amount of time, after which it will wake up and repeate the procedure.

## Setup
Use the files InfluxDBCredentials_template.h and WIFICredentials_template.h to create two files, InfluxDBCredentials.h and WIFICredentials.h, respectively. In these files, specify the credentials for your WIFI and the information needed for your InfluxDB client.

## Notes
This setup has so far only been tested with InfluxDB 1, not InfluxDB2.
