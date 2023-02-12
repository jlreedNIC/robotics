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

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

# what actions program uses
import irobot_create_msgs
from irobot_create_msgs.action import Undock, Dock, DriveDistance, RotateAngle 

# fancier feedback and goal reports
# look into the GoalInfo
from action_msgs.msg import GoalStatus


class Robot(Node):
    def __init__(self, namespace:str):
        super().__init__('random_walk')

        self._namespace = namespace # robot name

        self.result = None          # goal result
        self._action_client = None  # the single action client to spin on for the robot

        self.position = {'x': 0, 'y':0}     # x,y position of robot
        self.direction = 0                  # current rotation of robot
        self.home = {'x': 0, 'y':0}         # x,y position of dock

        self.random_count = 0               # random walk counter
    
    def send_goal(self, action_type, action_name:str, goal):

        print("\n")
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create action client with goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}')

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal
        send_goal_future = self._action_client.send_goal_async(goal, self.feedback_callback)

        # add done callbackfor response
        send_goal_future.add_done_callback(self.goal_response_callback)

        # spin node until goal completed
        self.result = None
        while self.result == None:
            rclpy.spin_once(self)
        
        # goal completed
        self.get_logger().warning(f"{action_name} action done")
    
    def feedback_callback(self, feedback):
        # self.get_logger().info(f'received feedback: {feedback}')
        pass
        # print(type(feedback))
        # print(dir(feedback))
        # need to update x,y position of robot
        # need to update rotation of robot
        # handle bump responses here??

    def goal_response_callback(self, future):
        """get goal response and deal with it

        :param _type_ future: _description_
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED")
            return
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.result = future.result().result
        status = future.result().status

        # get updated position and rotation
        try:
            # pos = DriveDistance.Result.pose
            pos = self.result.pose
            x = pos.pose.position.x # needed
            y = pos.pose.position.y # needed
            z = pos.pose.position.z

            quat_x = pos.pose.orientation.x
            quat_y = pos.pose.orientation.y
            quat_z = pos.pose.orientation.z # needed
            quat_w = pos.pose.orientation.w
            print(f'current position: ({x:.3f}, {y:.3f}, {z:.3f})')
            print(f'current rotation: ({quat_x:.3f}, {quat_y:.3f}, {quat_z:.3f}, {quat_w:.3f})')
        except Exception as e:
            # print(f"error: {e}")
            pass

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
    
    def bot_crashed(self):
        print("robot crashed and it doesn't know what to do yet")

def main():
    rclpy.init()
    bot = Robot("create3_0620")

    # undock
    cur_goal = Undock.Goal()
    bot.send_goal(Undock,'undock', cur_goal)

    # drive forward
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 1.0
    bot.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # turn 45deg
    cur_goal = RotateAngle.Goal()
    cur_goal.angle = 3.14159/4
    bot.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # drive 0.5m
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 0.5
    bot.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # rotate 135deg
    cur_goal = RotateAngle.Goal()
    cur_goal.angle = 3.14159*135/180
    bot.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # drive 0.6m
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 0.6
    bot.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # # rotate 180 deg
    # cur_goal = RotateAngle.Goal()
    # cur_goal.angle = 3.14159
    # bot.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # # drive .6m
    # cur_goal = DriveDistance.Goal()
    # cur_goal.distance = 0.6
    # bot.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # dock
    cur_goal = Dock.Goal()
    bot.send_goal(Dock, 'dock', cur_goal)

    # shut down node
    rclpy.shutdown()

if __name__ == '__main__':
    print("--PYTHON SCRIPT--")
    main()