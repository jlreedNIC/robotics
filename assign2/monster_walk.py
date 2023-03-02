#----------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     2/12/23 & 2/27/23
# @class    Robotics
#
# @desc     This program will send goals to the Create3 robot so the 
#           robot will do a random walk, handle bump responses, and return back 
#           to it's base.
#
#           This iteration is based off of Jacob's code in that it uses keyboard
#           input to start the program. The previous iteration was having problems
#           in multiple areas - executor, spinning, etc
#
# ------------------------------------------


import rclpy
from rclpy.node import Node

from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, Dock, NavigateToPosition
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.srv import ResetPose

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import RLock
from rclpy.executors import MultiThreadedExecutor

import random, math, time


# To help with Multithreading
lock = RLock()

class Monster(Node):
    """
    Class to coordinate actions and subscriptions
    """

    def __init__(self, namespace):
        super().__init__('monster_hunting')

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        cb_Subscripion = MutuallyExclusiveCallbackGroup()
        #cb_Action = cb_Subscripion
        self.cb_action =MutuallyExclusiveCallbackGroup()

        # Subscription to Hazards, the callback function attached only looks for bumper hits
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data,callback_group=cb_Subscripion)

        self._action_client = None          # reuse action client
        self._namespace = namespace

        # Variables
        self._goal_uuid = None          # for goal handling in mult. threads

        self.counter = 0                # for random walk iteration counting
        self.max_iter = 10              # maximum walk iterations

        # in order to reset pose
        self._reset_pose = self.create_client( ResetPose, f'/{self._namespace}/reset_pose')
        self.req = ResetPose.Request()

        self.pose = PoseStamped()       # current location of robot
        self.home = PoseStamped()       # home location of robot
        

    # -------------------------------------------------------------------------
    # generic action client and node functions

    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and if its
        a 'bump' message, cancel the current action. 

        For this to work, make sure you have:
        ros__parameters:
            reflexes_enabled: false
        in your Application Configuration Parameters File!!!
        '''

        # If it wasn't doing anything, there's nothing to cancel
        if self._goal_uuid is None:
            return

        # msg.detections is an array of HazardDetection from HazardDetectionVectors.
        # Other types can be gotten from HazardDetection.msg
        for detection in msg.detections:
            if detection.type == 1:   #If it is a bump
                self.get_logger().error('HAZARD DETECTED')

                with lock: # Make this the only thing happening
                    self.get_logger().warning('CANCELING GOAL')           
                    self._goal_uuid.cancel_goal_async()

                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        pass

                    self.get_logger().info("Goal canceled")

                    if self.counter >= self.max_iter:
                        self.counter = self.max_iter-1

                    # print("sleeping before starting back up")
                    time.sleep(1)
                
                self.get_logger().warning("Starting back up sequence")
                self.back_up()

    def send_generic_goal(self, action_type, action_name:str, goal):
        """
        function to send any goal async

        :param action_type: imported from irobot_create_messages
        :param action_name: string that is used with namespace
        :param goal: the goal object for action
        """
        print("\n")
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create action client with goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}', callback_group= self.cb_action)

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal in a lock and wait for send to finish
        with lock:
            send_goal_future = self._action_client.send_goal_async(goal) 
            while not send_goal_future.done():
                pass
            
            # set goal uuid so we know we have a goal in progress
            self._goal_uuid = send_goal_future.result()
        
        # make sure _goal_uuid is not none before accessing status
        while self._goal_uuid is not None and self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            # wait until status is set to something
            pass

        # make sure _goal_uuid is not none before accessing status
        self.get_logger().warning("Goal is in progress")
        while self._goal_uuid is not None and self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            if self._goal_uuid is not None and self._goal_uuid.status is GoalStatus.STATUS_CANCELED:
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass

        self.get_logger().info("Goal completed!")

        # get result object for positioning
        try:
            get_result_future = self._goal_uuid.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            # print(f"error getting result: {e}")
            pass
        
        # reset goal uuid to none after action is completed
        with lock:
            # print("reset goal uuid")
            self._goal_uuid = None

        # goal completed
        self.get_logger().warning(f"{action_name} action done")
        
    def get_result_callback(self, future):
        """
        a callback that will grab the position out of the result

        :param future: future passed in for get result
        """
        try:
            # print("goal has now completed, and getting result")
            result = future.result().result
        except Exception as e:
            # print("no result: {e}")
            pass

        # get updated position and rotation
        try:
            # pos = DriveDistance.Result.pose
            self.pose = result.pose
            
            print(f'current pose: ({self.pose.pose.position.x}, {self.pose.pose.position.y})')
        except Exception as e:
            # print(f"error getting pose: {e}")
            pass

    # --------------------------------------------------------------

    def start_hunting(self):
        """
        runs start up then start the random walk
        """

        self.start_up()
        # time.sleep(1)

        try:
            get_result_future = self._goal_uuid.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            # print(f"error getting result: {e}")
            pass

        
        self.home = self.pose
        self.home.pose.position.x -= .5
        print(f'home pose set to {self.home}')

        self.random_walk()

        self.go_home()
        
    def start_up(self):
        """
        undock and then move forward 1m
        """

        self._reset_pose.call(self.req) # reset pose
        
        # undock
        goal = Undock.Goal()
        self.send_generic_goal(Undock, 'undock', goal)

        # drive forward 1m
        goal = DriveDistance.Goal()
        goal.distance = 1.0
        self.send_generic_goal(DriveDistance, 'drive_distance', goal)
    
    def random_walk(self):
        """
        robot goes through loop of : rotate random angle, move forward .75m. 
        If 10 iterations have been done, go home
        """
        
        while self.counter < self.max_iter:
            print(f'in random walk loop counter: {self.counter}')

            # rotate random
            rand_angle = random.randrange(0,360)
            rand_angle = math.radians(rand_angle)
            cur_goal = RotateAngle.Goal()
            cur_goal.angle = rand_angle # from 0 to 360 deg
            self.send_generic_goal(RotateAngle, 'rotate_angle', cur_goal)

            # walk forward .75m
            cur_goal = DriveDistance.Goal()
            cur_goal.distance = .75
            self.send_generic_goal(DriveDistance, 'drive_distance', cur_goal)
            
            # with lock: # put in lock because accessing self.counter??
            with lock:
                # print("in lock for incrementing counter")
                self.counter += 1
            
            time.sleep(1)

        if self.counter >= self.max_iter:
            # go home
            self.go_home()
    
    def go_home(self):
        print("monster is looking for home")

        print(f'home pose: ({self.home.pose.position.x}, {self.home.pose.position.y})')
        print(f'current pose: ({self.pose.pose.position.x}, {self.pose.pose.position.y})')

        try:
            goal = NavigateToPosition.Goal()
            goal.goal_pose = self.home
            self.send_generic_goal(NavigateToPosition, 'navigate_to_position', goal)
        except Exception as e:
            # print(f'error with navigate: {e}')
            pass

        # have to send goal synchronously
        goal = Dock.Goal()
        self._action_client = ActionClient(node=self, action_type=Dock, action_name=f'/{self._namespace}/dock', callback_group=self.cb_action)
        self._action_client.send_goal(goal)

    def back_up(self):
        """
        called in the cancel callback
        rotate 180 deg and move .75m to go backward
        """
        self.get_logger().info('Starting back up process...')

        # rotate 180 deg
        goal = RotateAngle.Goal()
        goal.angle = math.pi # 180 deg
        self.send_generic_goal(RotateAngle, 'rotate_angle', goal)

        # walk forward .75m
        goal = DriveDistance.Goal()
        goal.distance = .75
        self.send_generic_goal(DriveDistance, 'drive_distance', goal)


if __name__ == '__main__':
    rclpy.init()

    namespace = 'create3_0620'
    m = Monster(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(m)

    keycom = KeyCommander([
        (KeyCode(char='r'), m.start_hunting),
        ])

    print("r: Start hunting")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        exec.shutdown()
        print("Destroying Monster Node")
        m.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()
