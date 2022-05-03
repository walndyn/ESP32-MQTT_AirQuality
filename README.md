# ESP32-MQTT_AirQuality
An ESP32 which monitors the air quality, temperature and other things (currently door status) and publishes it to your desired MQTT broker.

This is a small project of mine which monitors the air quality in my room (>>> temperature, humidity, CO2 and TVOC) with a DHT22 and a SGP30 sensor. It publishes it via MQTT to my MQTT broker (currently a Raspberry Pi 4). This then stores it in InfluxDB and presents it in Grafana.
In addition to that I wanted to monitor my door, count the people in the room and therefore switch on & off the lights in my room. So I implemented that as well. For the future I want to add a second ESP32 module across my room to have a more averaged and precise output. And maybe I will open & close my window based on the air quality.

For the project I use VS Code with PlatformIO because it´s just way more convenient than the Arduino IDE in my opinion.
I know the Code probably isn´t perfect because I´m still in the learning, but so far it works good enough.
Feel free to further improve my code :)



*In order to make things work you need to add a "credentials.h" file with your credentials to the src folder 
