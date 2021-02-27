import paho.mqtt.client as mqtt
import json
import requests
from datetime import datetime
import os

GET_URL = "http://vm-iot-school-2020-1.westeurope.cloudapp.azure.com//api/telemetry/measurement"
POST_URL = "http://vm-iot-school-2020-1.westeurope.cloudapp.azure.com/api/telemetry/measurement"

def on_connect(client, userdata, flags, rc):
    print("Connected to the broker")

def on_subscribe(client, userdata, mis, granted_ops):
    print("We are subscribed")

def on_publish(client, userdata, mid):
    print("We are publishing...")

def on_message(client, userdata, msg):
    print("topic: {}, message: {}".format(msg.topic, msg.payload))
    handle_mqtt_data(msg.topic, msg.payload)

def handle_mqtt_data(topic, payload):
    print("Handling mqtt data")
    json_data = dict()

    try:
        if "temperature" in topic:
            json_data["SensorName"] = "temperature"
            json_data["SensorValue"] = str(float(payload))

        elif "pulse" in topic:
            json_data["SensorName"] = "pulse"
            json_data["SensorValue"] = str(float(payload))

        elif "battery" in topic:
            json_data["SensorName"] = "battery"
            json_data["SensorValue"] = str(float(payload))

        else:
            print("Received an unknown message")

    except Exception as e:
        print("Encountered an error ", e)

    json_data["CreatedOn"] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    json_data["DeviceId"] = "1"
    post_data(json_data)

    # json_string = json.dumps(json_data)

    '''with open("mylogfile.log", 'w') as file:
        file.write(json_string)'''

def post_data(json_data):
    r = requests.post(url=POST_URL, json=json_data)
    if(r.status_code == 200):
        print("Posted data successfully")

    else:
        print("Error " + str(r.status_code))

def get_data(json_data):
    r = requests.get(url=GET_URL, json=json_data)
    if(r.status_code == 200):
        data = r.json()
        print(data)

    else:
        print("Error " + str(r.status_code))


if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_publish = on_publish
    client.on_message = on_message

    client.connect('localhost', 1883, 60)
    client.subscribe("algebra/iot/grga/#")

    client.loop_forever()
