#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import pygame
import math

class PygameROSController:
    def __init__(self):
        rospy.init_node('pygame_ros_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.pose = None
        self.yaw = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.previous_positions = []

        
        pygame.init()
        self.window_size = 600
        self.window_center = self.window_size // 2
        self.window = pygame.display.set_mode((self.window_size, self.window_size))
        pygame.display.set_caption('Husky Odom Graph and Keyboard Control')
        self.window.fill(pygame.Color(0, 0, 0))
        self.yellow = pygame.Color(245, 210, 0)
        pygame.display.update()

        self.clock = pygame.time.Clock()

    def odom_callback(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

        x = int((self.pose.position.x * 50) + self.window_center)
        y = int((self.pose.position.y * 50) + self.window_center)
        self.previous_positions.append((x, y))

    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown('Quit event received')
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.linear_speed += 1
                elif event.key == pygame.K_DOWN:
                    self.linear_speed -= 1
                elif event.key == pygame.K_LEFT:
                    self.angular_speed += 1
                elif event.key == pygame.K_RIGHT:
                    self.angular_speed -= 1
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    self.linear_speed = 0.0
                elif event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    self.angular_speed = 0.0

    def update_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def render(self):
        self.window.fill(pygame.Color(0, 0, 0))

        if self.pose is not None:
            for i in range(1, len(self.previous_positions)):
                pygame.draw.line(self.window, self.yellow, self.previous_positions[i - 1], self.previous_positions[i], 2)

            x, y = self.previous_positions[-1]
            font = pygame.font.Font(None, 36)
            text = font.render(f"Position: ({self.pose.position.x:.2f}, {self.pose.position.y:.2f})", True, (255, 255, 255))
            self.window.blit(text, (20, 20))
            orientation_deg = math.degrees(self.yaw)
            text = font.render(f"Orientation: {orientation_deg:.2f} degrees", True, (255, 255, 255))
            self.window.blit(text, (20, 60))

        pygame.display.flip()

    def run(self):
        while not rospy.is_shutdown():
            self.process_events()
            self.update_cmd_vel()
            self.render()
            self.clock.tick(30)

if __name__ == '__main__':
    try:
        controller = PygameROSController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
