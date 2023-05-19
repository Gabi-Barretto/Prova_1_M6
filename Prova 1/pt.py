#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from collections import deque
import math
from time import sleep

MAX_DIFF = 0.1

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # Publisher no topico cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # Subscriber no topico pose
        self.subscription_ = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription_

        self.twist_msg_ = Twist()
        self.goal_position = deque()
        self.going_back = []
        self.back = False
        self.current_position = [0, 0, 0]

        #Coordenadas propostas, lembrando que o próprio turtlesim não alcança elas........

        self.goal_position.append([0.5, 0.0])
        self.goal_position.append([0.0, 0.5])
        self.goal_position.append([0.5, 0.0])
        self.goal_position.append([0.0, 1.0])
        self.goal_position.append([1.0, 0.0])

        # self.goal_position.append([6.0, 6.0])
        # self.goal_position.append([8.0, 8.0])

        

    def pose_callback(self, msg):
        # Pose da tartaruga
        x = msg.x
        y = msg.y
        theta = msg.theta
        self.current_position = [x, y, theta]

        self.get_logger().info(f"Turtle pose - x: {x}, y: {y}, theta: {theta}")
        self.move_to_next_point()

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def move_to_next_point(self):
        
        next_point = self.goal_position[0]
        self.get_logger().info(f"next_point: {next_point}")
        
        goal_x = next_point[0]
        goal_y = next_point[1]

        msg = Twist()

        x_diff = goal_x - self.current_position[0]
        y_diff = goal_y - self.current_position[1]

        if self.distance(self.current_position[0], self.current_position[1], goal_x, goal_y) <= MAX_DIFF:
            self.going_back.append(next_point)
            msg.linear.x, msg.linear.y = 0.0, 0.0

            #Condição para não tertarmos retirar o valor de uma pilha com o método da lista que era utilizado inicialmente
            if self.back == False:
                self.goal_position.popleft()
            
            sleep(1)
            
            #Condição para a tartaruga voltar ao ponto inicial da fila
            if len(self.goal_position) == 0:
                self.goal_position = self.going_back.copy()
                self.back = True
                
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.5 if y_diff > 0 else -0.5
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        else:
            msg.linear.x = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init()
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
