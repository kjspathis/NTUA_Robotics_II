#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        """DECLARE CONSTANTS"""

        self.con1 = 0.0775

        self.con2 = 0.342543

        self.con3 = 0.075983

        self.con4 = 0.096978
        pass

    def compute_jacobian(self, r_joints_array):
        """ GET THE VALUES FROM self """
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)

        """------------------------------------DEPENDENCIES FOR THE J_1x------------------------------------- """

        """ DEPENDENCIES FOR THE J_11 """

        or1_11 = - con1*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)
        or2_11 = - con2*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)
        or3_11 = - con3*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        or4_11 = - con4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        or5_11 = - l3*(c1*s3 + c2*c3*s1) - l2*s1*s2

        J_11 = or1_11 + or2_11 + or3_11 + or4_11 + or5_11

        """ DEPENDENCIES FOR THE J_12 """
        or1_12 = + con1*(c1*c2*s4 - c1*c3*c4*s2)
        or2_12 = - con2*(c1*c2*c4 + c1*c3*s2*s4)
        or3_12 = - con3*(s6*(c1*c2*c4 + c1*c3*s2*s4) - c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5))
        or4_12 = - con4*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5))
        or5_12 = + l2*c1*c2 - l3*c1*c3*s2

        J_12 = or1_12 + or2_12 + or3_12 + or4_12 + or5_12

        """ DEPENDENCIES FOR THE J_13 """
        or1_13 = - con1*c4*(c3*s1 + c1*c2*s3)
        or2_13 = - con2*s4*(c3*s1 + c1*c2*s3)
        or3_13 = - con3*(c6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) + s4*s6*(c3*s1 + c1*c2*s3))
        or4_13 = + con4*(s6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3))
        or5_13 = - l3*(c3*s1 + c1*c2*s3)

        J_13 = or1_13 + or2_13 + or3_13 + or4_13 + or5_13

        """ DEPENDENCIES FOR THE J_14 """
        or1_14 = + con1*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)
        or2_14 = - con2*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)
        or3_14 = - con3*(s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2))
        or4_14 = - con4*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2))

        J_14 = or1_14 + or2_14 + or3_14 + or4_14

        """ DEPENDENCIES FOR THE J_15 """

        J_15 = (con3*c6 - con4*s6)*(c3*c5*s1 + c1*c2*c5*s3 - c1*s2*s4*s5 + c4*s1*s3*s5 - c1*c2*c3*c4*s5)

        """ DEPENDENCIES FOR THE J_16 """
        or3_16 =  - con3*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))
        or4_16 =  + con4*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))

        J_16 = or3_16 + or4_16

        """ DEPENDENCIES FOR THE J_17 """

        J_17 = 0

        """------------------------------------DEPENDENCIES FOR THE J_2x------------------------------------- """

        """ DEPENDENCIES FOR THE J_21 """
        or1_21 = - con1*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)
        or2_21 = - con2*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)
        or3_21 = - con3*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))
        or4_21 = - con4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))
        or5_21 = - l3*(s1*s3 - c1*c2*c3)

        J_21 = or1_21 + or2_21 + or3_21 + or4_21 + or5_21

        """ DEPENDENCIES FOR THE J_22 """
        or1_22 = + con1*(c2*s1*s4 - c3*c4*s1*s2)
        or2_22 = - con2*(c2*c4*s1 + c3*s1*s2*s4)
        or3_22 = - con3*(s6*(c2*c4*s1 + c3*s1*s2*s4) - c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5))
        or4_22 = - con4*(c6*(c2*c4*s1 + c3*s1*s2*s4) + s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5))
        or5_22 = + l2*c2*s1 - l3*c3*s1*s2

        J_22 = or1_22 + or2_22 + or3_22 + or4_22 + or5_22

        """ DEPENDENCIES FOR THE J_23 """
        or1_23 = + con1*c4*(c1*c3 - c2*s1*s3)
        or2_23 = + con2*s4*(c1*c3 - c2*s1*s3)
        or3_23 = + con3*(c6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) + s4*s6*(c1*c3 - c2*s1*s3))
        or4_23 = - con4*(s6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3))
        or5_23 = l3*(c1*c3 - c2*s1*s3)

        J_23 = or1_23 + or2_23 + or3_23 + or4_23 + or5_23

        """ DEPENDENCIES FOR THE J_24 """
        or1_24 = - con1*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)
        or2_24 = + con2*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)
        or3_24 = + con3*(s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2))
        or4_24 = + con4*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2))

        J_24 = or1_24 + or2_24 + or3_24 + or4_24

        """ DEPENDENCIES FOR THE J_25 """

        J_25 = -(con3*c6 - con4*s6)*(c1*c3*c5 - c2*c5*s1*s3 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5)

        """ DEPENDENCIES FOR THE J_26 """
        or3_26 = + con3*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        or4_26 = - con4*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))

        J_26 = or3_26 + or4_26

        """ DEPENDENCIES FOR THE J_27 """

        J_27 = 0


        """------------------------------------DEPENDENCIES FOR THE J_3x------------------------------------- """

        """ DEPENDENCIES FOR THE J_31 """

        J_31 = 0

        """ DEPENDENCIES FOR THE J_32 """
        or1_32 = - con1*(s2*s4 + c2*c3*c4)
        or2_32 = + con2*(c4*s2 - c2*c3*s4)
        or3_32 = - con3*(c6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) - s6*(c4*s2 - c2*c3*s4))
        or4_32 = + con4*(s6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) + c6*(c4*s2 - c2*c3*s4))
        or5_32 = - l2*s2 - l3*c2*c3

        J_32 = or1_32 + or2_32 + or3_32 + or4_32 + or5_32

        """ DEPENDENCIES FOR THE J_33 """

        J_33 = s2*(l3*s3 + con1*c4*s3 + con2*s3*s4 - con3*c3*c6*s5 + con4*c6*s3*s4 + con4*c3*s5*s6 + con3*s3*s4*s6 + con3*c4*c5*c6*s3 - con4*c4*c5*s3*s6)

        """ DEPENDENCIES FOR THE J_34 """
        or1_34 = + con1*(c2*c4 + c3*s2*s4)
        or2_34 = + con2*(c2*s4 - c3*c4*s2)
        or3_34 = + con3*(s6*(c2*s4 - c3*c4*s2) + c5*c6*(c2*c4 + c3*s2*s4))
        or4_34 = + con4*(c6*(c2*s4 - c3*c4*s2) - c5*s6*(c2*c4 + c3*s2*s4))

        J_34 = or1_34 + or2_34 +or3_34 + or4_24

        """ DEPENDENCIES FOR THE J_35 """

        J_35 = -(con3*c6 - con4*s6)*(c5*s2*s3 + c2*s4*s5 - c3*c4*s2*s5)

        """ DEPENDENCIES FOR THE J_36 """
        or3_36 = - con3*(s6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))
        or4_36 = - con4*(c6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) - s6*(c2*c4 + c3*s2*s4))

        J_36 = or3_36 + or4_36

        """ DEPENDENCIES FOR THE J_37 """

        J_37 = 0


        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J





    def tf_A01(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)

        tf = np.matrix([[c1 , -s1 , 0 , 0],\
                        [s1 , c1 , 0 , 0],\
                        [0 , 0 , 1 , l1],\
                        [0 , 0 , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A12 = np.matrix([[c2 , -s2 , 0 , 0],\
                            [0 , 0 , 1 , 0],\
                            [-s2 , c2 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A23 = np.matrix([[c3 , -s3 , 0 , 0],\
                            [0 , 0 , -1 , -l2],\
                            [s3 , c3 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A34 = np.matrix([[c4 , -s4 , 0 , l3],\
                            [0 , 0 , -1 , 0],\
                            [0 , 0 , 1 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5


        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A45 = np.matrix([[c5 , -s5 , 0 , l4s],\
                            [0 , 0 , -1 , -l4c],\
                            [s5 , c5 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A56 = np.matrix([[c6 , -s6 , 0 , 0],\
                            [0 , 0 , -1 , 0],\
                            [s6 , c6 , 1 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        con1 = self.con1
        con2 = self.con2
        con3 = self.con3
        con4 = self.con4
        l4s=con1
        l4c=con2
        l5s=con3
        l5c=con4

        """ DEFINE THE ANGLE JOINTS """
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        """ CREATE THE SINEs & COSINEs """
        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)
        tf_A67 = np.matrix([[c7 , -s7 , 0 , l5s],\
                            [0 , 0 , 1 , l5c],\
                            [-s7 , -c7 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
