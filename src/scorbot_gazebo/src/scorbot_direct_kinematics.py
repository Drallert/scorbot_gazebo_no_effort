#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
import roslib 


# Kinematics based on techreport "Drawing using the Scorbot-ER VII Manipulator Arm"  
# by Luke Cole, Adam Ferenc Nagy-Sochacki and Jonathan Symonds (2007)
# See: https://www.lukecole.name/doc/reports/drawing_using_the_scorbot_manipulator_arm.pdf



# (θ1,θ2,θ3,θ4) Input angles are expressed in radians
# (x,y,z) output position is expressed in millimeters
def direct_kinematics (theta1, theta2, theta3, theta4):
    # Distance are expressed in meters.
    d1 = 0.358  # Distance from base center (0,0,0) rotation (1) to shoulder/body center
    d2 = 0      # ?? Distance from center of the base to center of the shoulder/body axis
    a1 = 0.050     # Distance from shoulder/body center to shoulder/body joint (2)
    a2 = 0.300    # Distance from shoulder/body joint to elbow/arm joint (3)
    a3 = 0.250    # Distance from elbow/arm joint to pitch/forearm joint (4)
    a4 = 0.212    # End efector (gripper) length 

    # theta1 (θ1) = base rotation angle (1)
    # theta2 (θ2) = shoulder/body rotation angle (2)
    # theta3 (θ3) = elbow/arm rotation angle (3)
    # theta4 (θ4) = pitch/forearm rotation angle (4)

    c1 = math.cos (theta1)
    c2 = math.cos (theta2)
    c23 = math.cos (theta2 + theta3)
    c234 = math.cos (theta2 + theta3 + theta4)
    s1 = math.sin (theta1)

    # x:
    x = a4*c1*c234 + c1*c23*a3 + c1*c2*a2 + c1*a1 + s1*d2

     
    # y:
    y = s1*c23*a3 + s1*c2*a2 + s1*a1 + c1*d2

    s2 = math.sin (theta2)
    s23 = math.sin (theta2 + theta3)

    # z:
    z = -s23*a3 - s2*a2 + d1

    result = []
    result.append(x)
    result.append(y)
    result.append(z)


    return result

def move_angles_(theta1, theta2, theta3, theta4):
    pub_base = rospy.Publisher('/scorbot/base_position_controller/command', Float64, queue_size=10)
    pub_shoulder = rospy.Publisher('/scorbot/shoulder_position_controller/command', Float64, queue_size=10)
    pub_elbow = rospy.Publisher('/scorbot/elbow_position_controller/command', Float64, queue_size=10)
    pub_pitch = rospy.Publisher('/scorbot/pitch_position_controller/command', Float64, queue_size=10)
    #pub_roll = rospy.Publisher('/scorbot/roll_position_controller/command', Float64, queue_size=10)
    pub_base.publish (theta1)
    pub_shoulder.publish (theta2)
    pub_elbow.publish (theta3)
    pub_pitch.publish (theta4)
    #pub_roll.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))




if __name__ == '__main__':

    try:
        a1 = 0  #x0 = 0.8119999999   # 0.812 gives error (singular point: working area limit)
        a2 = 0  #y0 = 0
        a3 = 0  #z0 = 0.358
        a4 = 0
                                    # 01 solution
        #b1 = 0                      #x1 = 0.750
        #b2 = -0.5401742497052632    #y1 = 0
        #b3 = 0.9355365085259083     #z1 = 0.300
        #b4 = -0.39536225882064513   

                                    # 02 solution
        b1 = 0                      #x1 = 0.750
        b2 = 0.6616072627919588    #y1 = 0
        b3 = 1.1324677292534087     #z1 = 0.300
        b4 = -0.2232786652504709   

        

        print ("Begin example ...")
        data0 = direct_kinematics (a1,a2,a3,a4)
        data1 = direct_kinematics (b1,b2,b3,b4)
        print ( 'Scorbot end-effector (x,y,z) position for angles: theta1: ' + str(a1) + ' theta2: ' + str(a2) + ' theta3: ' + str(a3) + ' theta4: ' + str(a4))
        print (data0)
        print ( 'Scorbot end-effector (x,y,z) position for angles: theta1' + str(b1) + ' theta2: ' + str(b2) + ' theta3: ' + str(b3) + ' theta4: ' + str(b4) )
        print (data1)
        
        rospy.init_node('simple_angle_mover')
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            move_angles_ (a1,a2,a3,a4)

            rate.sleep()
            move_angles_ (b1,b2,b3,b4) 

            rate.sleep()           

    except rospy.ROSInterruptException:
        pass


