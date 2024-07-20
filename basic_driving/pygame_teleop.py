#!/usr/bin/env python3

#A simple teleop node using Python
#arrow keys are pressed to increase linear and angular velocity
#press s to stop the robot
#press esc to kill the node

import rospy
from geometry_msgs.msg import Twist
import pygame


class PygameTeleop:
    def __init__(self):
        rospy.init_node('pygame_teleop')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linear_speed= 0.0
        self.angular_speed = 0.0
 
        pygame.init()
        self.window_size = (800, 600)
        self.screen = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption('Robot Teleoperation')
        self.font = pygame.font.Font(None, 36)

        self.clock = pygame.time.Clock()

    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown('Quit event received')
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    if self.linear_speed<=0.75:
                        self.linear_speed += 0.25
                elif event.key == pygame.K_DOWN:
                    if self.linear_speed>=-0.75:
                        self.linear_speed -= 0.25
                elif event.key == pygame.K_LEFT:
                    if self.angular_speed<=0.75:
                        self.angular_speed += 0.25
                elif event.key == pygame.K_RIGHT:
                    if self.angular_speed>=-0.75:
                        self.angular_speed -= 0.25
                elif event.key == pygame.K_s:  
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                elif event.key == pygame.K_ESCAPE:
                    rospy.signal_shutdown('ESC key pressed')

    def update_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def render(self):
        self.screen.fill((0, 0, 0))

        commanded_text = self.font.render(
            f"Commanded - Linear: {self.linear_speed:.2f} m/s, Angular: {self.angular_speed:.2f} rad/s",
            True, (255, 255, 255))

        self.screen.blit(commanded_text, (20, 20))

        pygame.display.flip()

    def run(self):
        while not rospy.is_shutdown():
            self.process_events()
            self.update_cmd_vel()
            self.render()
            self.clock.tick(30)

if __name__ == '__main__':
    try:
        teleop = PygameTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
