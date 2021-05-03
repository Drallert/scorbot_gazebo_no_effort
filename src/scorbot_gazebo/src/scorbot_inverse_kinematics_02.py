#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
import roslib 

#Global variables:


# Kinematics based on techreport "Drawing using the Scorbot-ER VII Manipulator Arm".  
# Authors: Luke Cole, Adam Ferenc Nagy-Sochacki and Jonathan Symonds (2007)
# https://www.lukecole.name/doc/reports/drawing_using_the_scorbot_manipulator_arm.pdf

# Auxiliary function:
def atan2 (y, x):
    if (x>0):
        return math.atan (y/x)
    elif (y>=0 and x<0):
        return math.atan (y/x) + math.pi
    elif (y<0 and x<0):
        return math.atan (y/x) - math.pi
    elif (y>0 and x==0):
        return (math.pi)/2
    elif (y<0 and x==0):
        return (-math.pi)/2
    else :
        return 0



# (x,y,z) input positions are expressed in millimeters
# (θ1,θ2,θ3,θ4) Output angles are expressed in radians
def inverse_kinematics (x, y, z):
    # Distance are expressed in meters.
    # Data is taken from Intelitek Scorbot ER VII user's manual (page 20) 
    d1 = 0.358  # Distance from base center (0,0,0) rotation (1) to shoulder/body center
    d2 = 0      # Value 0 is wrong: distance from center of base to shoulder join
    a1 = 0.050     # Distance from shoulder/body center to shoulder/body joint (2)
    a2 = 0.300    # Distance from shoulder/body joint to elbow/arm joint (3)
    a3 = 0.250    # Distance from elbow/arm joint to pitch/forearm joint (4)
    a4 = 0.212    # End efector (gripper) length 

    # theta1 (θ1) = base rotation angle (1)
    # theta2 (θ2) = shoulder/body rotation angle (2)
    # theta3 (θ3) = elbow/arm rotation angle (3)
    # theta4 (θ4) = pitch/forearm rotation angle (4)

    # theta1:
    #theta1 = math.atan2 (y,x) - math.atan2 (d2, math.sqrt(x*x+y*y+z*z))
    theta1 = atan2 (y,x) - atan2 (d2, math.sqrt(x*x+y*y+z*z))
    print ("theta1: ")
    print (theta1)
    print ()

    # theta3:
    #theta3 =  math.acos ( ( pow(math.cos(theta1)*x + math.sin(theta1)*y - a1, 2) + pow (d1-z, 2) - a2*a2 - a3*a3 )  / 2*a2*a3 )
    term0 = math.cos(theta1)*x + math.sin(theta1)*y - a1
    term1 = math.pow (term0,2)
    term2 = math.pow (d1-z,2) - a2*a2 - a3*a3
    term3 =  2*a2*a3
    term4 = term1+term2
    #print ("term3:")
    #print (term3)
    #print ("term4:")
    #print (term4)
    term5 = term4/term3
    #print ("term5:")
    #print(term5)
    #print (math.acos (2.8542933323173334))
    term6 = math.acos (term5)
    theta3 = term6

    # theta2:
    #theta2 =  2*math.atan ( (d1-z + math.sqrt( (d1-z)*(d1-z) + math.pow (math.cos(theta1)*x + math.sin(theta1)*y - a1, 2) + math.pow (a3*math.cos(theta3) +a2, 2) )) / (math.cos(theta1)*x + math.sin(theta1)*y - a1 + a3*math.cos(theta3) +a2)  ) 
    term1 = d1-z
    term2 = (d1-z)*(d1-z)
    term3 = math.cos (theta1)*x + math.sin (theta1)*y - a1
    term4 = a3*math.cos (theta3) + a2
    term5 = math.sqrt (term2 + math.pow(term3,2) + math.pow(term4,2) )
    term6 = term3+term4
    term7 = (term1 + term5) / term6
    term8 = 2*math.atan (term7)
    theta2 = term8
    

    # theta4:
    theta4 = math.pi/2 - theta2 - theta3

    result = []
    result.append(theta1)
    result.append(theta2)
    result.append(theta3)
    result.append(theta4)
    return result



def move_angles(theta1, theta2, theta3, theta4):
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



def transform_URDF_coordinates_to_DH_coordinates (x, y, z):
    # To transform D-H coordinates, Zurdf axis must be first 180º rotated. 
    # Then the YURDF axis must be rotated -90º rotated. 
    new_x = z
    new_y = -y
    new_z = x

    result = []
    result.append(new_x)
    result.append(new_y)
    result.append(new_z)
    return result




if __name__ == '__main__':

    try:
        x0 = 0.8119999999   # 0.8119999999   # 0.812 gives error (singular point: working area limit)
        y0 = 0
        z0 = 0.358  #358

        x1 = 0.750
        y1 = 0
        z1 = 0.300

        print ("Begin example ...")

        data0 = transform_URDF_coordinates_to_DH_coordinates (x0, y0, z0)
        #print ("Transformed P0 coordinates: ")
        #print (data0[0]) 
        #print (data0[1])
        #print (data0[2])
        #print ()
        data00 = inverse_kinematics (data0[0],data0[1],data0[2])
        #data00 = inverse_kinematics (x0,y0,z0)
        
        data1 = transform_URDF_coordinates_to_DH_coordinates (x1, y1, z1)
        #print ("Transformed P1 coordinates: ")
        #print (data1[0]) 
        #print (data1[1])
        #print (data1[2])
        #print ()
        data11 = inverse_kinematics (data1[0],data1[1],data1[2])
        #data11 = inverse_kinematics (x1,y1,z1)

        print ( 'Scorbot angles for x0: ' + str(x0) + ' y: ' + str(y0) + ' z: ' + str(z0) )
        print (data00)
        print()
        print ( 'Scorbot angles for x1: ' + str(x1) + ' y: ' + str(y1) + ' z: ' + str(z1) )
        print (data11)
        
        rospy.init_node('simple_angle_mover')
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            #move_angles (data00[0],0,data00[2],data00[3]) 
            move_angles (data00[0],data00[1],data00[2],data00[3]) 
            #move_angles (0.000000, -0.811667, 1.567459, -0.755791); 
            #move_angles (0.000000, -0.540174, 0.935537, -0.395362); 

            rate.sleep()
            move_angles (data11[0],data11[1],data11[2],data11[3])
            #move_angles (0.000000, -0.692448, 1.565213, -0.872764);
            #move_angles (0.000000, -0.055050, 0.121125, -0.066075);

            rate.sleep()           

    except rospy.ROSInterruptException:
        pass


