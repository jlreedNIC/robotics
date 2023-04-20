# used for testing mqtt and multithreading
import time
import datetime
from threading import RLock


mqtt_err = False
robot_name = "dj"
start_sync = False
lock = RLock()

try:
    import paho.mqtt.client as paho
    from paho import mqtt
except Exception as e:
    print("Error: please install 'paho.mqtt.client' module.")
    mqtt_err = True

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
    message_string = msg.payload.decode("ascii")
    
    print(f'\ntopic: {msg.topic}')
    print(f'{message_string}')

    # if msg.topic == "synchronize" and robot_name not in message_string:
    #     print('received message from other robot')
    #     with lock:
    #         print('obtained lock and set start_sync')
    #         global start_sync
    #         start_sync = True
    #         # print(start_sync)
    #     # start_sync_func(client)
    # else:
    #     print('received message from self')

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

    clientname = client._client_id
    clientname = clientname.decode(("ascii"))
    new_msg = f"{clientname}: {message}"
    client.publish(topic, new_msg, qos=1)



def print_instr() :
    print("\nWhat would you like to do?")
    print(" (1) Send message \n (0) Exit \n")

#---------main program----------

if not mqtt_err:
    client = connect_local_MQTT()
    client.subscribe("#")

    client.loop_start()
    start_sync = False

    print(f'started listening program for {robot_name}')
    
    print(f'I just executed this movement and am now waiting')

    while not start_sync:
        # client.loop_read()
        # print(f'sync: {start_sync}')
        pass

    print(f'now starting synchronized swimming program')

    client.loop_stop()

    # client.loop_forever()
    