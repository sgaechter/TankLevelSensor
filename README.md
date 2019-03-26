# TankLevelSensor
Tank Level sensor with Waterproof Ultrasonic Module

Keep attention: 
The Tank must be as cubic as possible, otherwise there is some additional programming needed to avoid measorement faults. 

Functionality:
On triggering the Script the Sensor gets triggered and makes 20 Measurements, calculates the average and prints the result.
The Script is designed for Python 3 on RaspberryPi  

---------------------
MQTT_Tanklevelsensor.ino

This Version ist built for ESP8266 with MQTT Protocol.
On the ESP there is the Possibility to connect a Climate Sensor (DHT22) and the Ultrasonic Sensor.
The Values are pushed by MQTT to a broker like Mosquito on Raspberry pi or simmilar (No Login credentials are implemented yet)

Check the in Code comments to configure for your needs.
