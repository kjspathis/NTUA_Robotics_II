#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        #Publish new values for the essay
        self.pub1 = rospy.Publisher("/velocity_Linear_x", Float64, queue_size=100)
        self.pub2 = rospy.Publisher("/velocity_Linear_y", Float64, queue_size=100)
        self.pub3 = rospy.Publisher("/velocity_Angular_z", Float64, queue_size=100)
        self.pub4 = rospy.Publisher("/roll", Float64, queue_size=100)
        self.pub5 = rospy.Publisher("/pitch", Float64, queue_size=100)
        self.pub6 = rospy.Publisher("/yaw", Float64, queue_size=100)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)
    def polyonymo(self,x0,xf,tf):
        a0=x0
        a1=0.000
        a2=(3/(tf**2))*(xf-x0)
        a3=-(2/(tf**3))*(xf-x0)
        polyonymo=np.poly1d([a3,a2,a1,a0])

        #p1d=np.array([p1dx,p1dy,p1dz]).T
        return polyonymo
    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        print("aaaa",self.velocity)
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")


        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
        d_safe=0.4
        traj1=self.polyonymo(0,2,0.5)


        while not rospy.is_shutdown():
            "sonars_output"
            sonar_front = self.sonar_F.range # and so on...
            sonar_frontleft=self.sonar_FL.range
            sonar_frontright=self.sonar_FR.range
            sonar_left=self.sonar_L.range
            sonar_right=self.sonar_R.range
            "robot's place in the workspace"
            (roll, pitch, self.imu_yaw) = quaternion_to_euler(self.imu.orientation.w, self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z)
            angular_velocity=self.imu.angular_velocity
            linear_acceleration=self.imu.linear_acceleration
            left_wheel_pos=self.joint_states.position[0]
            right_wheel_pos=self.joint_states.position[1]
            left_wheel_vel=self.joint_states.velocity[0]
            right_wheel_vel=self.joint_states.velocity[1]

            """
            INSERT YOUR MAIN CODE HERE
            self.velocity.linear.x = ...
            self.velocity.angular.z = ...
            """

            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9


            #trajecory planning
            #traj1=self.polyonymo(0,2,0.5)
            derivative=np.polyder(traj1)
            xdot = np.polyval(derivative,dt)
            self.velocity.linear.x = xdot
            self.velocity.linear.y=self.imu_yaw * xdot
            klish=atan2(self.velocity.linear.y,xdot)
            self.velocity.angular.z =0.0
            x = np.polyval(traj1,dt)
            y = self.imu_yaw * x
            if (sonar_front < d_safe or sonar_right<d_safe-0.2 or (sonar_frontright* cos(klish)-0.018)<d_safe-0.2 ) :
                print("sonar_left")
                x = 0
                y = 0
                self.velocity.linear.x = 0.0
                self.velocity.linear.y=0.0
                turn1=self.polyonymo(0,1.6,0.5)
                derivative=np.polyder(turn1)
                ang_vel= np.polyval(derivative,dt)
                self.velocity.angular.z =-ang_vel
                #self.velocity_pub.publish(self.velocity)



            # Publish the new joint's angular positions
            self.pub1.publish(self.velocity.linear.x)
            self.pub2.publish(self.velocity.linear.y)
            self.pub3.publish(self.velocity.angular.z)
            self.pub4.publish(roll)
            self.pub5.publish(pitch)
            self.pub6.publish(self.imu_yaw)

            self.velocity_pub.publish(self.velocity)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
