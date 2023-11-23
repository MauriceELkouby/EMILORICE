import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO # General Purpose Input/Output library
# Define the MQTT broker and topic
#Subscribes to the appropriate MQTT topic
broker_address = "localhost"  # Replace with the Raspberry Pi's IP if not running locally
topic = "tank/heater"  #MQTT topic

# Set up the GPIO mode
GPIO.setmode(GPIO.BCM)

#pin 18 as output
pin=18

GPIO.setup(pin, GPIO.OUT)


# Callback functions for MQTT client
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(topic)

def on_message(client, userdata, message):
    print(f"Received message on topic '{message.topic}': {message.payload.decode()}")
    if message.payload.decode() == "on": #if on, turn on the LED
         GPIO.output(pin, 1)
    if message.payload.decode() == "off": #if off, turn off the LED
        GPIO.output(pin, 0)

# Create an MQTT client
client = mqtt.Client()

# Set up callback functions
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address)

# Start the MQTT client loop to receive messages
client.loop_forever()
