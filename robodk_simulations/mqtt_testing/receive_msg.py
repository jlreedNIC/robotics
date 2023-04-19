# https://pypi.org/project/paho-mqtt/

# weird install path for python
# import sys
# sys.path.append("/home/jreed/.local/lib/python3.7/site-packages")
import time
import datetime
# import playsound

# print("Starting messaging program...")

mqtt_err = False
robot_name = "dj"
start_sync = False

try:
    import paho.mqtt.client as paho
    from paho import mqtt
except Exception as e:
    # print(f"error with import: {e}")
    print("Error: please install 'paho.mqtt.client' module.")
    # print("Please enter: python3 -m pip install paho-mqtt")
    mqtt_err = True
else:
    print("paho module is installed.")

# message_queue = []

# callbacks

def on_connect(client, userdata, flags, rc, properties=None) :
    if rc==0:
        print(f'connected successfuly code={rc}')

def on_subscribe(client, userdata, mid, granted_qos, properties=None) :
    print(f'Subscribed: {mid} {granted_qos}')

def on_publish(client, userdata, mid, properties=None) :
    print(f'mid: {mid}')

def on_message(client, userdata, msg) :
    print("message received")
    # print(f"topic: {msg.topic} message:\n{msg.payload}")
    # timestamp = datetime.datetime.now()
    message_string = msg.payload.decode("ascii")
    # temp_message = [message_string, timestamp]
    # message_queue.append(temp_message)
    print(f'\ntopic: {msg.topic}')
    print(f'{message_string}')
    # playsound.playsound("proximityalert_ep.mp3")

    if msg.topic == "synchronize" and robot_name not in message_string:
        print('received message from other robot')
        start_sync = True
        start_sync_func(client)
    else:
        print('received message from self')

    # print_messages()
    # print_instr()

def start_sync_func(client):
    print('executing start sync swimming')
    print('now disconnecting')
    client.disconnect()

def on_disconnect(client, userdata, rc, properties=None) :
    if rc == paho.MQTT_ERR_SUCCESS  :
        print("Disconnected successfully")
    else :
        print("Unexpected disconnect")

# ---------- connecting

def connect_local_MQTT() :
    print("start connecting")
    client_id = robot_name # put in config file
    localMQTT = paho.Client(client_id)

    # set up call backs
    localMQTT.on_connect = on_connect
    ip_addr = "453e51c107604519a5fd673fd39f1313.s1.eu.hivemq.cloud"
    port = 8883
    username = "student"
    password = "cs383_students"
    # connect
    localMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
    localMQTT.username_pw_set(username, password)
    localMQTT.connect(host=ip_addr, port=port)

    # more callbacks
    localMQTT.on_message = on_message
    localMQTT.on_subscribe = on_subscribe
    localMQTT.on_disconnect = on_disconnect
    localMQTT.on_publish = on_publish

    return localMQTT    

def publishMessage(topic, message) :
    print("starting to publish")
    # client = connect_local_MQTT()

    # client.loop_start()
    clientname = client._client_id
    clientname = clientname.decode(("ascii"))
    new_msg = f"{clientname}: {message}"
    client.publish(topic, new_msg, qos=1)

    # client.loop_stop()

    # client.disconnect()


def print_instr() :
    print("\nWhat would you like to do?")
    print(" (1) Send message \n (0) Exit \n")

#---------main program----------

if not mqtt_err:
    client = connect_local_MQTT()
    client.subscribe("#")

    # client.loop_start()
    start_sync = False

    print(f'started listening program for {robot_name}')
    
    print(f'I just executed this movement and am now waiting')

    # while not start_sync:
    #     # client.loop_read()
    #     # print(f'sync: {start_sync}')
    #     pass

    # print(f'now starting synchronized swimming program')

    # client.loop_stop()

    client.loop_forever()
    