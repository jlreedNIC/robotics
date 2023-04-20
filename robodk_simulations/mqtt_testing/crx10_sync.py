
# imports
from threading import RLock
from robodk.robolink import *           # import the robolink library
RDK = Robolink()                        # connect to the RoboDK API (RoboDK starts if it has not started)


# common variables - config
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

# ---------------------------
#
# mqtt callbacks
#
# ---------------------------

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

    if msg.topic == "synchronize" and robot_name not in message_string:
        print('received message from other robot')
        with lock:
            global start_sync
            start_sync = True
    else:
        print('received message from self')

def on_disconnect(client, userdata, rc, properties=None) :
    if rc == paho.MQTT_ERR_SUCCESS  :
        print("Disconnected successfully")
    else :
        print("Unexpected disconnect")

# ---------- connecting to mqtt broker

def connect_local_MQTT() :
    print("start connecting")
    client_id = robot_name 
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

    return localMQTT    

def publishMessage(topic, message) :
    print("starting to publish")

    clientname = client._client_id
    clientname = clientname.decode(("ascii"))
    new_msg = f"{clientname}: {message}"
    client.publish(topic, new_msg, qos=1)

# -----------------
#
# robodk code
# 
# -----------------

def start_robodk(robolink, runmode, frame_name):
    robot = robolink.Item('', ITEM_TYPE_ROBOT)
    robot.Connect()
    robolink.setRunMode(runmode)

    frame = robolink.Item(frame_name, ITEM_TYPE_FRAME)
    robot.setPoseFrame(frame)

    return robot 

def move_robot(robot, tgt_names, lin_speed=1500, j_speed=180, loop=1):
    
    for i in range(0, len(tgt_names)*loop):             # move through targets
        robot.setSpeed(speed_linear = lin_speed, speed_joints = j_speed)                  # set speed
        target = RDK.Item(tgt_names[i%len(tgt_names)], ITEM_TYPE_TARGET)
        robot.MoveJ(target)

def wave(robot):
    move_robot(robot, ['home'], 1000, 30)                # go to home position

    move_robot(robot, ['wave_down', 'wave_up'], loop=2) # wave up and down 3 times

    move_robot(robot, ['home'], 1000, 30)                # go to home position

def spin_and_wave(robot):
    move_robot(robot, ['squat_down', 'quarter_up', 'mostly_up', 'all_up'])      # spin robot up
    move_robot(robot, ['all_up', 'mostly_up', 'quarter_up', 'squat_down'])      # spin robot down

    move_robot(robot, ['squat_wave_left', 'squat_wave_right'], loop=2)          # robot waves from squat

    move_robot(robot, ['home'], 1000, j_speed=30)  

#---------main program----------

robot = start_robodk(RDK, RUNMODE_RUN_ROBOT, 'refPoint')

if not mqtt_err:
    client = connect_local_MQTT()
    client.subscribe("#")

    client.loop_start()

    print(f'started listening program for {robot_name}')
    
    # put robot code here
    wave(robot)
    spin_and_wave(robot)

    publishMessage('synchronize', 'ready')

    while not start_sync:
        # wait for other robot to be done
        pass

    # synchronized robot code here
    move_robot(robot, ['home'])
    move_robot(robot, ['wave_down', 'wave_up'], loop=2)
    move_robot(robot, ['sync_out', 'sync_up'])
    move_robot(robot, ['home'])
    
    client.loop_stop()
    client.disconnect()
