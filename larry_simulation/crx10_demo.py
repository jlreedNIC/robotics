import time

from robodk.robolink import *           # import the robolink library
RDK = Robolink()                        # connect to the RoboDK API (RoboDK starts if it has not started)

robots = RDK.ItemList(ITEM_TYPE_ROBOT)  # get robot list?
print(robots)

def start_robodk(robolink, runmode, frame_name):
    robot = robolink.Item('', ITEM_TYPE_ROBOT)
    robot.Connect()
    robolink.setRunMode(runmode)

    frame = robolink.Item(frame_name, ITEM_TYPE_FRAME)
    robot.setPoseFrame(frame)

    return robot 

def regular_wave(robot):
    print('robot is waving from home position')

    tgt_names = ['wave_down', 'wave_up']

    # put robot in home position
    robot.setSpeed(1000, 5)
    target = RDK.Item('home', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, 10)

    for i in range(0, len(tgt_names)*3):
        target = RDK.Item(tgt_names[i%2], ITEM_TYPE_TARGET)
        robot.MoveJ(target)
	
    # put robot in home position
    robot.setSpeed(1000, 5)
    target = RDK.Item('home', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

def spin_up(robot):
    print('robot is spinning up')

    tgt_names = ['squat_down', 'quarter_up', 'mostly_up', 'all_up']

    robot.setSpeed(1500, 10)

    for i in range(0, len(tgt_names)):
        target = RDK.Item(tgt_names[i%2], ITEM_TYPE_TARGET)
        robot.MoveJ(target)

def spin_down(robot):
    print('robot is spinning down')

    tgt_names = ['all_up', 'mostly_up', 'quarter_up', 'squat_down']

    robot.setSpeed(1500, 10)

    for i in range(0, len(tgt_names)):
        target = RDK.Item(tgt_names[i%2], ITEM_TYPE_TARGET)
        robot.MoveJ(target)

def squat_wave(robot):
    print('robot is waving from squat')

    tgt_names = ['squat_wave_left', 'squat_wave_right']

    robot.setSpeed(1500, 10)

    for i in range(0, len(tgt_names)*3):
        target = RDK.Item(tgt_names[i%2], ITEM_TYPE_TARGET)
        robot.MoveJ(target)
	

# actual program

robot = start_robodk(RDK, RUNMODE_SIMULATE, 'refPoint')

regular_wave(robot)
spin_up(robot)
spin_down(robot)
squat_wave(robot)


target = RDK.Item('home', ITEM_TYPE_TARGET)
robot.MoveJ(target)




