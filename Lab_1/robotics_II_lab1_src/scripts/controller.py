#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        #print(self.model_states_sub)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        print(self.model_states)
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)
    def polyonymo(self,y0,yf,tf):
        a0=y0
        a1=0.000
        a2=(3/(tf**2))*(yf-y0)
        a3=-(2/(tf**3))*(yf-y0)
        polyonymo=np.poly1d([a3,a2,a1,a0])

        #p1d=np.array([p1dx,p1dy,p1dz]).T
        return polyonymo




    def publish(self):

        # set configuration
        self.joint_angpos = np.array([0, 0.75, 0, 1.5, 0, 0.75, 0],dtype='float')
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")



        pass

        #Oscilation_period
        T=0.5
        green_obstacle=self.model_states.pose[1].position.y
        red_obstacle=self.model_states.pose[2].position.y

        #trajetory for py(0.2->0.2)
        #traj1=self.polyonymo(,0.200,T/4)[0]

        #trajetory for py(0.2->-0.2)
        #traj1=self.polyonymo(0.200,-0.200,T/2,t)[0]

        #trajetory for py(-0.2->0.2)
        #traj1=self.polyonymo(-0.200,0.200,T/2,t)[0]


        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()


        K1=0.5
        Kc=5
        d_safe=0.15

        Pa=np.array([0.6043,0.200,0.1508])
        Pb=np.array([0.6043,-0.200,0.1508])
        p1dy=self.polyonymo(0.000,red_obstacle,T/4)

        while not rospy.is_shutdown():

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A01 = self.kinematics.tf_A01(self.joint_angpos)
            self.A02 = self.kinematics.tf_A02(self.joint_angpos)
            self.A03 = self.kinematics.tf_A03(self.joint_angpos)
            self.A04 = self.kinematics.tf_A04(self.joint_angpos)
            self.A05 = self.kinematics.tf_A05(self.joint_angpos)
            self.A06 = self.kinematics.tf_A06(self.joint_angpos)
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)

            #1h ypoergasia

            rostime_now = rospy.get_rostime()
            time_ref = rostime_now.to_nsec()
            time = np.abs(time_now-time_ref)/1e9

            if (self.A07[1,3]>=0.2000):
                rostime_now = rospy.get_rostime()
                time_ref = rostime_now.to_nsec()
                time = np.abs(time_now-time_ref)/1e9
                p1dy=self.polyonymo(red_obstacle,green_obstacle,T/2)
            if (self.A07[1,3]<=-0.200):
                rostime_now = rospy.get_rostime()
                time_ref = rostime_now.to_nsec()
                time = np.abs(time_now-time_ref)/1e9
                p1dy=self.polyonymo(green_obstacle,red_obstacle,T/2)
            derivative=np.polyder(p1dy)


            pydot = np.polyval(derivative,time)
            pxdot=0
            pzdot=0
            p1d_dot=np.array([pxdot,pydot,pzdot])
            print('CHECKPOINT 1',p1d_dot)


            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)
            # pseudoinverse jacobian
            pinvJ = pinv(J)


            #INSERT YOUR MAIN CODE HERE
            #self.joint_angvel[0] = ...

            identity_matrix=np.identity(7)

            #synarthsh krithrioy:V(q)=min(p(q)-o) ....mono ston y aksona.......ypologizw tis merikes paragwgoys ws pros kathe arthrwsh
            q1=self.joint_angpos[0]
            q2=self.joint_angpos[1]
            q3=self.joint_angpos[2]
            py_robot=5.25*(cos(q1)*sin(q3) + cos(q3)*cos(q2)*sin(q1)) - 29.3*sin(q1)*sin(q2)
            partial_q1=5.25*(-sin(q1)*sin(q3) + cos(q3)*cos(q2)*cos(q1)) - 29.3*cos(q1)*sin(q2)
            partial_q2=5.25*(-sin(q2)*cos(q3)*sin(q1))- 29.3*cos(q2)*sin(q1)
            partial_q3=5.25*(cos(q1)*cos(q3) - cos(q2)*sin(q3)*sin(q1))
            py_robot2=-29.3*(sin(q1)*sin(q2))
            partial2_q1=-29.3*(cos(q1)*sin(q2))
            partial2_q2=-29.3*(sin(q1)*cos(q2))

            """ksi1=0
            ksi2=0
            ksi3=0
            ksi4=0
            ksi5=0
            ksi6=0
            ksi7=0
            ksi=(np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float'))"""

            if (np.abs(py_robot - green_obstacle) < d_safe) :
                ksi1=partial_q1*(green_obstacle - py_robot -d_safe)
                ksi2=partial_q2*(green_obstacle - py_robot +d_safe)
                ksi3=partial_q3*(green_obstacle - py_robot -d_safe)
                ksi4=0.0
                ksi5=0.0
                ksi6=0.0
                ksi7=0.0
                ksi=np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float')

            if (np.abs(py_robot - red_obstacle) < d_safe) :
               ksi1=partial_q1*(red_obstacle - py_robot +d_safe )
               ksi2=partial_q2*(red_obstacle - py_robot -d_safe)
               ksi3=partial_q3*(red_obstacle - py_robot +d_safe )
               ksi4=0
               ksi5=0
               ksi6=0
               ksi7=0
               ksi=(np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float'))
            """
            if (np.abs(py_robot2 - green_obstacle) < d_safe) :
                ksi1=partial2_q1*(green_obstacle - py_robot -d_safe )
                ksi2=partial2_q2*(green_obstacle - py_robot -d_safe)
                ksi3=0.0
                ksi4=0.0
                ksi5=0.0
                ksi6=0.0
                ksi7=0.0
                ksi=np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float')

            if (np.abs(py_robot2 - red_obstacle) < d_safe) :
               ksi1=partial2_q1*(red_obstacle - py_robot -d_safe )
               ksi2=partial2_q2*(red_obstacle - py_robot -d_safe)
               ksi3=0
               ksi4=0
               ksi5=0
               ksi6=0
               ksi7=0
               ksi=(np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float'))
            """
            if ((np.abs(py_robot - green_obstacle) >= d_safe) and (np.abs(py_robot - green_obstacle) >= d_safe) ):
                ksi1=0
                ksi2=0
                ksi3=0
                ksi4=0
                ksi5=0
                ksi6=0
                ksi7=0
                ksi=np.array([ksi1,ksi2,ksi3,ksi4,ksi5,ksi6,ksi7],dtype='float')



            q1dot = np.asarray(np.dot(np.asarray(pinvJ),p1d_dot))
            #q1dot=q1dot
            JplusJ = np.asarray(np.dot(pinvJ,J))
            nsp= np.identity(7)-JplusJ
            q2dot = np.asarray(np.dot(nsp,ksi))
            #q2dot=q2dot
            self.joint_angvel=q1dot + Kc*q2dot
            #self.joint_angvel=self.joint_angvel.T
            #q1dot = np.asarray(np.dot(np.asarray(pinvJ),p1d_dot))
            #self.joint_angvel=q1dot
            print('CHECKPOINT 2',q1dot)


            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9
            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )
            #self.joint_angpos=np.array([self.joint_angpos[0],self.joint_angpos[1],self.joint_angpos[2],self.joint_angpos[3],self.joint_angpos[4],self.joint_angpos[5],self.joint_angpos[6]],dtype=float)
            #self.joint_angpos=float(self.joint_angpos)

            # Publish the new joint's angular positions
            #self.joint_angpos=np.float64(self.joint_angpos*pi/180)
            #print(self.joint_angpos[0])
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            print('CHECKPOINT 3',self.joint_angpos)
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
