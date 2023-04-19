#  demo created by Jordan Reed
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

    move_robot(robot, ['home'], 1000, j_speed=30)                                                 # robot to home position

# main prog
robot = start_robodk(RDK, RUNMODE_RUN_ROBOT, 'refPoint')

wave(robot)
spin_and_wave(robot)