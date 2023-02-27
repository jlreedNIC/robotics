#----------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     2/12/23
# @class    Robotics
#
# @desc     This program will send goals to the Create3 robot so the 
#           robot will do a random walk, handle bump responses, and return back 
#           to it's base.
#
# ------------------------------------------

import math     # for more accurate pi
import random   # for random

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

# qos subscribing stuff for hazard detection
from rclpy.qos import qos_profile_sensor_data

# multithreading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import Lock
from rclpy.executors import MultiThreadedExecutor

# what actions program uses
import irobot_create_msgs
from irobot_create_msgs.action import Undock, Dock, DriveDistance, RotateAngle, NavigateToPosition
from irobot_create_msgs.msg import HazardDetectionVector

# fancier feedback and goal reports
# look into the GoalInfo
from action_msgs.msg._goal_status import GoalStatus

lock = Lock()
# exec = MultiThreadedExecutor(2)

class Robot(Node):
    def __init__(self, namespace:str):
        super().__init__('random_walk')
        # exec.add_node(self)

        self._namespace = namespace # robot name

        self.result = None          # goal result
        self._action_client = None  # the single action client to spin on for the robot
        self._goal_uuid = None      # tracking goal

        # callback groups handling multithreading and canceling goals
        self.cb_sub = MutuallyExclusiveCallbackGroup()
        self.cb_action = MutuallyExclusiveCallbackGroup()

        # subscription to hazard vector and callback to look for bumper
        # self.subscription = self.create_subscription(
        #     HazardDetectionVector, f'/{self._namespace}/hazard_detection', 
        #     self.listener_callback, qos_profile_sensor_data, callback_group=self.cb_sub)

        self.position = {'x': 0, 'y':0}     # x,y position of robot
        self.direction = 0                  # current rotation of robot
        self.home = {'x': 0, 'y':0}         # x,y position of dock

        self.random_count = 0               # random walk counter
    
    def listener_callback(self, msg):
        """every time subscription gets message, it checks to see what it was and if a goal needs to be canceled

        :param _type_ msg: _description_
        """
        # if no goal, then nothing
        if self._goal_uuid is None:
            return
        
        # check for bump
        for detection in msg.detections:
            if detection.type == 1: # if bump
                self.get_logger().warning('BUMP DETECTED')

                # lock out cancel goal to make sure nothing else is happening
                # will need to handle random_count and turn 180 and walk .75m
                with lock:
                    self.get_logger().warning('CANCELING CURRENT GOAL')
                    # handle = self._goal_uuid.cancel_goal_async()
                    # print(handle.result)

                    # while handle is not GoalStatus.STATUS_CANCELED:
                        # pass
                    # print("handle was canceled")

                    # loop until goal status return canceled
                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        # print(self._goal_uuid.status)
                        # print(f'goal status cancelled: {GoalStatus.STATUS_CANCELED}')
                        pass
                    self.get_logger().info('Goal canceled')

                    if self.random_count >= 20:
                        self.random_count = 19
                    # how to handle coordinate system?
                
                # back_up()

    def send_goal(self, action_type, action_name:str, goal):

        print("\n")
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create action client with goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}', callback_group=self.cb_action)

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal in a lock and wait for send to finish
        with lock:
            send_goal_future = self._action_client.send_goal_async(goal) #, self.feedback_callback)
            print(send_goal_future)
            while not send_goal_future.done():
                # print(f'{send_goal_future.result()}')
                pass
            
            # set goal uuid so we know we have a goal in progress
            self._goal_uuid = send_goal_future.result() ## ISN'T THIS SOMEWHERE ELSE??
            # print(self._goal_uuid)
        
        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            # wait until status is set to something
            pass

        # add done callbackfor response
        print("adding response callback")
        send_goal_future.add_done_callback(self.goal_response_callback)

        # spin node until goal completed
        self.result = None
        while self.result == None:
            print("still spinning node")
            rclpy.spin_once(self)
            # rclpy.spin_once(self, exec)
            # exec.spin_once()
        
        # reset goal uuid to none after action is completed
        with lock:
            self._goal_uuid = None

        # goal completed
        self.get_logger().warning(f"{action_name} action done")
    
    def feedback_callback(self, feedback):
        self.get_logger().info(f'received feedback: {feedback}')
        
        print(type(feedback))
        
        # print(dir(feedback))
        # need to update x,y position of robot
        # need to update rotation of robot
        # handle bump responses here??

        pass

    def goal_response_callback(self, future):
        """get goal response and deal with it

        :param _type_ future: _description_
        """
        print("received response from node, setting uuid...")
        goal_handle = future.result()
        # print(goal_handle)

        with lock:
            self._goal_uuid = goal_handle
            print(self._goal_uuid)

        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED")
            self.result = "rejected"
            return
        
        get_result_future = goal_handle.get_result_async()
        print("get result!")
        get_result_future.add_done_callback(self.get_result_callback)
        print("added callback")
    
    def get_result_callback(self, future):

        print("goal has now completed, and getting result")
        self.result = future.result().result
        status = future.result().status

        # get updated position and rotation
        try:
            # pos = DriveDistance.Result.pose
            pos = self.result.pose
            x = pos.pose.position.x # needed
            y = pos.pose.position.y # needed
            z = pos.pose.position.z

            self.position['x'] = x
            self.position['y'] = y

            quat_x = pos.pose.orientation.x
            quat_y = pos.pose.orientation.y
            quat_z = pos.pose.orientation.z # needed
            quat_w = pos.pose.orientation.w

            self.rotation = quat_z
            print(f'current position: ({self.position["x"]:.3f}, {self.position["y"]:.3f}')
            print(f'current rotation: ({self.rotation:.3f})')
        except Exception as e:
            # print(f"error: {e}")
            pass

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
            print(status)
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
    
    def bot_crashed(self):
        print("robot crashed and it doesn't know what to do yet")
        self.get_logger().info('Starting back up process...')

        # rotate 180 deg
        cur_goal = RotateAngle.Goal()
        cur_goal.angle = math.pi # 180 deg
        self.send_goal(RotateAngle, 'rotate_angle', cur_goal)

        # walk forward .75m
        cur_goal = DriveDistance.Goal()
        cur_goal.distance = .75
        self.send_goal(DriveDistance, 'drive_distance', cur_goal)

    def random_walk(self):
        if self.random_count < 20:
            self.start_walking()
        else:
            print("bot is ready to go home")
            self.go_home()
    
    def start_walking(self):
        while self.random_count < 20:
            print(f'in random walk loop counter: {self.random_count}')

            # rotate random
            rand_angle = random.randrange(0,360)
            rand_angle = math.radians(rand_angle)
            cur_goal = RotateAngle.Goal()
            cur_goal.angle = rand_angle # from 0 to 360 deg
            self.send_goal(RotateAngle, 'rotate_angle', cur_goal)

            # walk forward .75m
            cur_goal = DriveDistance.Goal()
            cur_goal.distance = .75
            self.send_goal(DriveDistance, 'drive_distance', cur_goal)
            
            with lock: # put in lock because accessing self.counter??
                self.random_count += 1
    
    def go_home(self):
        print("robot is supposed to be going home, but it doesn't know how to do that yet")
        print(f'current location: ({self.position["x"]}, {self.position["y"]}')

        # do math here to figure out how much to rotate and where to drive

        # cur_goal = DriveDistance.Goal()
        # cur_goal.distance = 1.0
        # self.send_goal(DriveDistance, 'drive_distance', cur_goal)

        cur_goal = Dock.Goal()
        self.send_goal(Dock, 'dock', cur_goal)

    def start(self):

        print("bot is waking up")
        # undock
        cur_goal = Undock.Goal()
        self.send_goal(Undock,'undock', cur_goal)

        cur_goal = DriveDistance.Goal()
        cur_goal.distance = 1.0
        self.send_goal(DriveDistance, 'drive_distance', cur_goal)

        print("bot is ready to go random!")

def main():
    rclpy.init()
    bot = Robot("create3_0620")

    # exec = MultiThreadedExecutor(2) # defines number of threads
    # exec.add_node(bot)
    

    print(f"goal aborted: {GoalStatus.STATUS_ABORTED}")
    print(f"goal accepted: {GoalStatus.STATUS_ACCEPTED}")
    print(f"goal canceled: {GoalStatus.STATUS_CANCELED}")
    print(f"goal canceling: {GoalStatus.STATUS_CANCELING}")
    print(f"goal executing: {GoalStatus.STATUS_EXECUTING}")
    print(f"goal succeeded: {GoalStatus.STATUS_SUCCEEDED}")
    print(f"goal unkown: {GoalStatus.STATUS_UNKNOWN}")

    bot.start()

    bot.random_walk()

    

    # shut down node
    rclpy.shutdown()

if __name__ == '__main__':
    print("--PYTHON SCRIPT--")
    main()