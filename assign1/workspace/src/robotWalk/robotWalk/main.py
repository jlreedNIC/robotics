#----------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     Jan 30, 2023
# @class    Robotics

# @desc     This program will send goals to the Create3 robot so the 
#           robot will move forward 1m, turn 45 degree, move 
#           forward .5m, and return home.
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle

class MyNode(Node):
    def __init__(self, namespace:str):
        # call superclass init
        super().__init__('walker')

        print(f'constructing {namespace} node')
        self.namespace = namespace

        # need action clients for each command
        self.drive_ac = ActionClient(self, DriveDistance, f'/{self.namespace}/drive_distance')
        self.undock_ac = ActionClient(self, Undock, f'/{self.namespace}/undock')
        self.dock_ac = ActionClient(self, Dock, f'/{self.namespace}/dock')
        self.rotate_ac = ActionClient(self, RotateAngle, f'/{self.namespace}/rotate_angle')
    
    def undock_robot(self):
        print("now executing undock command")

        # wait for server
        self.get_logger().warning("waiting for server...")
        self.undock_ac.wait_for_server()

        # server available
        self.get_logger().warning("server available!")
        self.get_logger().warning("sending goal now...")

        # create goal object
        undock_goal = Undock.Goal()

        # send goal
        self.undock_ac.send_goal_async(undock_goal)
        self.get_logger().warning("robot is UNDOCKED")
    
    def dock_robot(self):
        print("now executing dock command")

        # wait for server
        self.get_logger().warning("waiting for server...")
        self.dock_ac.wait_for_server()

        # server available
        self.get_logger().warning("server available!")
        self.get_logger().warning("sending goal now...")

        # create goal object
        dock_goal = Dock.Goal()

        # send goal
        self.dock_ac.send_goal_async(dock_goal)
        self.get_logger().warning("robot is DOCKED")
    
    def drive(self):
        self.undock_robot()
        self.dock_robot()

def main():
    print("Hello there! Just testing out the first robot program")

    rclpy.init()
    node = MyNode("create3_0620")

    node.drive()

    rclpy.spin(node)
    rclpy.shutdown()