# Project IoT_Aquarium
December 7, 2023 

Emilio Jesus Chavero Suarez - 2251904
Maurice Elkouby - 

## Project description
The goal of this project is to create an automated aquarium capable of sensing elements such as temperature or water level and reacting automatically to changes. The project comprises an ESP32 connected to a temperature sensor, a TDS sensor, and an ultrasonic sensor. Additionally, there is a Raspberry Pi that serves as a remote controller and MQTT Broker.
The ESP32 reads the values and publishes them to the appropriate MQTT Topic. The Raspberry Pi runs several programs in the background, including control.py, which subscribes to the MQTT Topics and performs logic in accordance with the configuration in config.json; actor.py, which subscribes to the results topics and performs actions such as controlling relays; historian.py, which keeps track of the MQTT values published and creates a database with the values; and app.py, which creates a web interface to access the values from a browser.

## Reflections
Emilio: 
The project turned out to be more challenging than anticipated, as a few issues were encountered. The tank was originally supposed to have a heating element that is controlled by a relay. However, the heating element was faulty, with an internal short circuit that rendered it unusable. Nonetheless, the issue was solved by replacing the heater with an LED to simulate its function.
Another issue was that actor.py failed to recognize the topic for the heater being on. This problem occurred due to a mismatch between the result in config.json and the topic subscribed to in actor.py.
In general, the project was enjoyable to work on, and it taught me the basics of IoT systems. Through the project, I also gained experience with coding languages that I am unfamiliar with, such as HTML, CSS, and SQL.


Maurice: 
As previously stated, this projected turned out to be quite a handful. We ended up with a faulty heating element, which happened due to us not wanting to buy a high watt usage heating element and cutting the wire as we weren't sure how to be safe. I think switching to the LED was the best idea for our scenario as it provides a safe and easy way to simulate our system. We had different issues with the code aswell and with the transfer of data from the sensor to the historian, but through testing and researching we both learned and grew from these experiences and I we have a greater and stronger understanding of IOT systems. Ultimately, working on the project was amusing and when things worked it was exciting and really gave us a good feeling, I also enjoyed the use of multiple coding languages do build an intricate yet simple IOT system.   
