
import time

from robodk.robolink import *                  # import the robolink library
RDK = Robolink()                        # connect to the RoboDK API (RoboDK starts if it has not started

robots = RDK.ItemList(ITEM_TYPE_ROBOT)
print(robots)


# tool  = RDK.Item('SCHUNK- 1321170 Co-act EGP-C 40-N-N-KTOE_ 0')                # Get an item named Tool (name in the RoboDK station tree)

robot = RDK.Item('', ITEM_TYPE_ROBOT)   # Get the first available robot


robot.Connect()
RDK.setRunMode(RUNMODE_RUN_ROBOT)
#RDK.setRunMode(RUNMODE_SIMULATE)
#frame = RDK.ItemUserPick('Select a reference frame', ITEM_TYPE_FRAME)   # Promt the user to select a reference frame
frame = RDK.Item('Frame 2', ITEM_TYPE_FRAME) 

robot.setPoseFrame(frame)
# robot.setPoseTool(tool)

# robot.setSpeed(1500)
speed = [360]

for i in range(0,len(speed)):
	print(f"Run: {i} Speed: {speed[i]}")
	RDK.ShowMessage(message = f"Run: {i} Speed: {speed[i]}")

	robot.setSpeed(speed[i], speed[i])
	robot.setSpeedJoints(speed[i])

	target = RDK.Item('home', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	# target.setSpeed(speed[i])
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame

	time.sleep(1.0)

	target = RDK.Item('wave_up', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame

	target = RDK.Item('wave_down', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame

	

target = RDK.Item('home', ITEM_TYPE_TARGET)
robot.MoveJ(target)




