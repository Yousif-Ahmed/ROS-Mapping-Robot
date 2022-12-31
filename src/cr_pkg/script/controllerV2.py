#!/usr/bin/env python3

import rospy
import threading
import sys
import termios
import tty
from select import select
from geometry_msgs.msg import Twist

moveBindings = {
    'w':(1,0,0,0),
    'a':(0,0,0,1),
    'd':(0,0,0,-1),
    's':(-1,0,0,0),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('controller')
    print("Controller node started")

    speed = 1.0
    turn = 1.0
    speed_limit = 4.0
    turn_limit = 6.0
    repeat = 0.0
    key_timeout = 0.25
    twist_frame = ''

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

                # increase speed by 10% if the key is held down
                if key == 'w' or key == 's':
                    speed = min(speed_limit, speed * 1.1)
                elif key == 'a' or key == 'd':
                    turn = min(turn_limit, turn * 1.1)
                
                print(vels(speed,turn))
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0 and speed == 1.0 and turn == 1.0:
                    continue
                
                # decelerate if no key is pressed
                if x > 0:
                    x = max(0, x - 0.25)
                elif x < 0:
                    x = min(0, x + 0.25)
                
                if y > 0:
                    y = max(0, y - 0.25)
                elif y < 0:
                    y = min(0, y + 0.25)
                
                if z > 0:
                    z = max(0, z - 0.25)
                elif z < 0:
                    z = min(0, z + 0.25)
                
                if th > 0:
                    th = max(0, th - 0.25)
                elif th < 0:
                    th = min(0, th + 0.25)

                speed = 1.0
                turn = 1.0

                print(vels(speed,turn))

                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)