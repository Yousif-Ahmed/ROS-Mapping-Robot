import select
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import termios
import tty


def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])





class Controller:
    def __init__(self):
        self.pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        
        
    def process_keyboard_input(self, control_str):
        if control_str == 'w':
            self.move_forward()
        elif control_str == 's':
            self.move_backward()
        elif control_str == 'a':
            self.turn_left()
        elif control_str == 'd':
            self.turn_right()
        
        rospy.Timer(rospy.Duration(0.4), self.stop, oneshot=True)

        

        
    
    def move_forward(self):
        self.cmd_vel.linear.x += 1
        self.pub.publish(self.cmd_vel)

    
    def move_backward(self):
        self.cmd_vel.linear.x -= 1
        self.pub.publish(self.cmd_vel)
    
    def turn_left(self):
        self.cmd_vel.angular.z += 1
        self.pub.publish(self.cmd_vel)
    
    def turn_right(self):
        self.cmd_vel.angular.z -= 1
        self.pub.publish(self.cmd_vel)
    
    def stop(self, temp = None):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.pub.publish(self.cmd_vel)
    
    
    def read_inputs(self):
        # This function is supposed to be a non blocking function that read keyboard input and publish it
        # to the /cmd_vel topic
        
        while True:
            c = sys.stdin.read(1)

            
            if c == 'q':
                break
            self.process_keyboard_input(c)
    
    def non_blocking_read(self):
        # This function is supposed to be a non blocking function that read keyboard input and publish it
        # to the /cmd_vel topic
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                if isData():
                    c = sys.stdin.read(1)
                    if c == 'q':
                        break
                    self.process_keyboard_input(c)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)




if __name__ == '__main__':
    rospy.init_node('controller')
    print("Controller node started")
    controller = Controller()
    controller.non_blocking_read()