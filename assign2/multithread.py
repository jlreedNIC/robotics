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

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock
from rclpy.executors import MultiThreadedExecutor

import random, math, time


# To help with Multithreading
lock = Lock()

class Slash(Node):
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

        self._action_client = None
        self._namespace = namespace

        # Variables
        self._goal_uuid = None

        # random walk counter
        self.counter = 0

        self.x = 0
        self.y = 0
        self.quatz = 0

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
                self.get_logger().warning('HAZARD DETECTED')
                with lock: # Make this the only thing happening
                    self.get_logger().warning('CANCELING GOAL')           
                    self._goal_uuid.cancel_goal_async()

                    # Loop until the goal status returns canceled
                    print("entering loop while not canceled")
                    print(f"status: {self._goal_uuid.status} canceled: {GoalStatus.STATUS_CANCELED}")
                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED and self._goal_uuid.status is not None:
                        print("goal not canceled yet")
                        pass
                    # else:
                    #     while self._goal_uuid is not None:
                    #         pass  
                    print('Goal canceled.')

                    print("sleeping before starting back up")
                    time.sleep(2)
                
                self.back_up()


#--------------------Async send goal calls-----------------------------
    def sendDriveGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        
        with lock:
            drive_handle = self._drive_ac.send_goal_async(goal)
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal
            
            # Hold ID in case we need to cancel it
            self._goal_uuid = drive_handle.result() 

        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED:
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass
        
        with lock:
            # Reset the goal ID, nothing should be running
            self._goal_uuid = None 

    def send_generic_goal(self, action_type, action_name:str, goal):

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
            
        self.get_logger().warning("Goal is in progress")
        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            # wait until status is set to something
            pass

        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED:
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass

        self.get_logger().info("Goal completed!")
        # print(self._goal_uuid.status)

        get_result_future = self._goal_uuid.get_result_async()
        print("get result!")
        get_result_future.add_done_callback(self.get_result_callback)
        
        # reset goal uuid to none after action is completed
        with lock:
        # print("reset goal uuid")
            self._goal_uuid = None

        # goal completed
        self.get_logger().warning(f"{action_name} action done")

        
    def get_result_callback(self, future):

        # print("goal has now completed, and getting result")
        result = future.result().result
        status = future.result().status

        # get updated position and rotation
        try:
            # pos = DriveDistance.Result.pose
            pos = result.pose
            x = pos.pose.position.x # needed
            y = pos.pose.position.y # needed
            z = pos.pose.position.z

            self.x = x
            self.y = y
            # self.position['x'] = x
            # self.position['y'] = y

            quat_x = pos.pose.orientation.x
            quat_y = pos.pose.orientation.y
            quat_z = pos.pose.orientation.z # needed
            quat_w = pos.pose.orientation.w

            self.quatz = quat_z

            print(f'current position: ({self.x:.3f}, {self.y:.3f}')
            print(f'current rotation: ({self.quatz:.3f})')
        except Exception as e:
            # print(f"error: {e}")
            pass

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
            print(status)
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
#----------------------------------------------------------------------


    def drive_away(self):
        """
        Undocks robot and drives out a meter asynchronously
        """

        self.start_up()
        self.random_walk()
        

    def start_up(self):
        # undock
        goal = Undock.Goal()
        self.send_generic_goal(Undock, 'undock', goal)
        # undock_ac = ActionClient(self, Undock, f'/{self._namespace}/undock', callback_group=self.cb_action)
        # self.get_logger().warning("waiting for server")
        # undock_ac.wait_for_server()
        # self.get_logger().warning("sending goal")
        # undock_ac.send_goal(goal)
        # self.get_logger().info("UNDOCKED")

        # drive forward 1m
        goal = DriveDistance.Goal()
        goal.distance = 1.0
        self.send_generic_goal(DriveDistance, 'drive_distance', goal)
    
    def random_walk(self):
        max_iter = 3
        # if self.counter < max_iter:
            # do random
        while self.counter < max_iter:
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
                print("in lock for incrementing counter")
                self.counter += 1
            pass

        if self.counter >= max_iter:
            # go home
            print("monster is ready to go home now")
            self.go_home()
            pass
    
    def go_home(self):
        print("monster is looking for home")

        try:
            goal = NavigateToPosition.Goal()
            goal.goal_pose = PoseStamped(self.x, self.y)
            self.send_generic_goal(NavigateToPosition, "navigate_to_position", goal)
        except Exception as e:
            print(f'error with navigate: {e}')

        goal = Dock.Goal()
        self.send_generic_goal(Dock, 'dock', goal)

    def back_up(self):
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
    s = Slash(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(s)

    keycom = KeyCommander([
        (KeyCode(char='r'), s.drive_away),
        ])

    print("r: Start drive_away")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        exec.shutdown()
        print("Destroying Node")
        s.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()
