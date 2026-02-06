import rclpy
import time
from rclpy.node import Node
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from geometry_msgs.msg import Twist
import sys, tty, termios, select

dic_joints = {
# Head
"HS":{"name":"HeadYaw", "min":-2.0857, "max":2.0857},
"HUD":{"name":"HeadPitch", "min":-0.7068, "max":0.6371},
# left                                              
"LSUD":{"name":"LShoulderPitch", "min":-2.0857, "max": 2.0857},
"LSS":{"name":"LShoulderRoll", "min":0.0087, "max":1.5620},
"LES":{"name":"LElbowYaw", "min":-2.0857, "max":2.0857},
"LEUD":{"name":"LElbowRoll", "min":-1.5620, "max":-0.0087},
"LWS":{"name":"LWristYaw", "min":-1.8239, "max":1.8239},
"LH":{"name":"LHand", "min":0.0, "max":1.0},
# right
"RSUD":{"name":"RShoulderPitch", "min":-2.0857, "max":2.0857},
"RSS":{"name":"RShoulderRoll", "min":-1.5620, "max":-0.0087},
"RES":{"name":"RElbowYaw", "min":-2.0857, "max":2.0857},
"REUD":{"name":"RElbowRoll", "min":0.0087, "max":1.5620},
"RWS":{"name":"RWristYaw", "min":-1.8239, "max":1.8239},
"RH":{"name":"RHand", "min":0.0, "max":1.0},
# bottom
"BS":{"name":"HipRoll", "min":-0.5149, "max":0.5149},
"BUD":{"name":"HipPitch", "min":-1.0385, "max":1.0385},
"KUD":{"name":"KneePitch", "min":-0.5149, "max":0.5149}
} 
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [],[],0.1)
    if rlist:
        key = sys.stdin.read(3)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class JointControlNode(Node):
    def __init__(self):
        # super = above Node + __init__ = initialize
        super().__init__('joint_control_node')
        # ros2 pub => create_publisher in the python
        # /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed "{ => same meaning / 10 => buffer
        self.joint_pub = self.create_publisher(JointAnglesWithSpeed, '/joint_angles', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    # for control key board

    def move_base(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.vel_pub.publish(msg)

    def send_command(self, joint_name, angle):
        msg = JointAnglesWithSpeed()
        msg.joint_names = [joint_name]
        msg.joint_angles = [angle]
        msg.speed = 0.2
        msg.relative = 0
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info(f'Sending: {joint_name} to {angle}')

# rclpy initialization 
def main (args=None):
    settings = termios.tcgetattr(sys.stdin) # keyboard setting
    rclpy.init(args=args)
    node = JointControlNode()
    # My command
    while rclpy.ok(): # before crtl+c, It's working
        node.get_logger().info(f'You can choose Base : 1, Joint : 2, Finish : 3')
        try:
            mode_number = float(input("mode: "))
            if mode_number == 1:
                while True:
                    key = get_key(settings)
                    linear_x = 0.0
                    angular_z = 0.0
                    if key == '\x1b[A':   # Up
                        linear_x = 0.2
                      
                    elif key == '\x1b[B': # Down
                        linear_x = -0.2
                        
                    elif key == '\x1b[D': # Left
                        angular_z = 0.5
                        
                    elif key == '\x1b[C': # Right
                        angular_z = -0.5
                    elif 'q' in key:
                        node.move_base(0.0, 0.0)
                        break
                    else:
                        linear_x = 0.0
                        angular_z = 0.0
                    node.move_base(linear_x, angular_z)

            elif mode_number == 2:
                joint_name = input("Choose the Joint?: ")
                # check the name is right
                if joint_name not in dic_joints:
                    node.get_logger().info(f'Joint name is wrong')
                    continue
                target_name = dic_joints[joint_name]['name']
                target_angle_max = dic_joints[joint_name]["max"]
                target_angle_min = dic_joints[joint_name]["min"]
                while True:
                    try:
                        target_angle = float(input("Angle: "))   
                        if target_angle > target_angle_max or target_angle < target_angle_min:
                            node.get_logger().info(f'Angle is not correct')
                        else :
                            break
                    except ValueError:
                        node.get_logger().info(f'please input only number')
                # check the angle is right
                node.send_command(target_name, target_angle)
                time.sleep(1.0)
            elif mode_number == 3:
                break
            else:
                continue
        except ValueError:
            node.get_logger().info(f'please input only 1, 2, 3')

    # destroy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()