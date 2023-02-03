import rospy
import sys, select, os
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#import sys, select, os
velocity = 0
steering = 0
breakcontrol = 1
gear = 0
MAX_Velocity = 125

target_linear_vel = 0;
target_angular_vel = 0;

teleop_pub = rospy.Publisher('teleop_cmd_vel', Twist,queue_size=1)

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
CTRL-C to quit
"""

#Code for receiving input from the keyboard
def getkey():
        fd = sys.stdin.fileno()
        original_attributes = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
        return ch

#Code for displaying changed values through the terminal.
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


def teleop():
    global velocity,steering,breakcontrol,gear
    rospy.init_node('teleop', anonymous=True)
#    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rate = rospy.Rate(10) # 10hz
#    try:
    status = 0
    while not rospy.is_shutdown():
        key = getkey()
        if key == 'w':
            velocity = velocity + 5
            steering = 0 
            status = status + 1
        elif key == 's':
            velocity = 0
            #steering = 0
            status = status + 1
        elif key == 'a':
            steering = steering + 2
            status = status + 1
        elif key == 'd':
            steering = steering - 2
            status = status + 1
        elif key == 'x':
            velocity = velocity - 5
            steering = 0
            status = status + 1
        else:
            if (key == '\x03'):
                break
        pubmsg = Twist()
        if velocity >= MAX_Velocity:
            velocity = MAX_Velocity

        if velocity <= -MAX_Velocity:
            velocity = -MAX_Velocity
  
        pubmsg.linear.x = velocity
        pubmsg.angular.z = steering
        teleop_pub.publish(pubmsg)
        print('currently:\tlinear vel ' + str(velocity) + '\t angular vel '+ str(steering))
        "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
        #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
