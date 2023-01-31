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
        # self.get_logger().warning("robot is UNDOCKED")
    
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
        # self.get_logger().warning("robot is DOCKED")
    
    def drive_robot(self, drive_dist:float):
        print(f'starting drive command {drive_dist}m')

        # wait for server
        self.get_logger().warning("waiting for server")
        self.drive_ac.wait_for_server()

        # server avail
        self.get_logger().warning("server available. now sending goal")

        # create goal object
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = drive_dist

        # send goal
        self.drive_ac.send_goal_async(drive_goal)

        # create goal finished message?
        self.get_logger().info("goal sent and executing")
    
    def rotate_robot(self, angle:float):
        
        # wait for server
        self.get_logger().info("waiting for server")
        self.rotate_ac.wait_for_server()

        # server available
        self.get_logger().info("Server AVAILABLE!")

        # create goal object
        rotate_goal = RotateAngle.Goal()
        rotate_goal.angle = angle

        # send goal (async because wayland issue??)
        self.get_logger().info("sending goal now!")
        self.rotate_ac.send_goal(rotate_goal)

        # send complete message
        self.get_logger().info("goal sent!")
        
    
    def command(self):
        ''' robot will drive forward 1m, turn 45deg, drive .5m, turn 135deg, drive .6m'''
        self.undock_robot()
        self.drive_robot(1.0)
        self.rotate_robot(3.14159/4)
        self.drive_robot(.5)
        self.rotate_robot(3.14159*3/4)
        self.drive_robot(.6)
        self.dock_robot()

def main():
    print("Hello there! Just testing out the first robot program")

    rclpy.init()
    node = MyNode("create3_0620")

    node.command()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    print('code is run as python script, not package')
    main()