from sympy import *
import numpy as np 
import matplotlib.pyplot as plt
import sympy as sym

#Links of the Scorbot-ER
l1=35.85
l2=30
l3=35
l4=25.1

def WristComputation(px,py):
    '''This function computes the position of the wirst as the midle point of all possibilities
    Inputs are the two coordinates of the target postion (the two remaining after the base is turned. Output: Wrist position'''
    px=float(px)
    py=float(py)
    
    x,y= symbols('x,y')
    
    #This two data points define the intersection of the end-effector circle (centered in the target and radius of l4) and the circle centered in the upperbase of the robot and radius l2+l3
    circle_1 = Eq((x-px)**2+(y-py)**2,l4**2)
    circle_2= Eq(x**2+y**2,(l2+l3)**2)
    (x,y)=solve([circle_1,circle_2],(x,y))

    #print ("All the points in that curve are valid solutions for (wx,wy)")

    a=np.array([x[1]-py],dtype=np.float64)
    b=np.array([x[0]-px],dtype=np.float64)

    c1=np.arctan2(a,b)#* 180 / np.pi
    #print (c1* 180 / np.pi)

    a=np.array([y[1]-py],dtype=np.float64)
    b=np.array([y[0]-px],dtype=np.float64)

    c2=np.arctan2(a,b)#* 180 / np.pi
    #We set the angle values to positive values so we can then create the incremental array 
    if (c1<0): 
        c1=c1+2*np.pi
    elif (c2<0):
        c2=c2+2*np.pi

    #Now, c1 and c2 are positive angles (anti clock-wise reference) and we can build the incremental vector of angles. 
    if (c1>c2): 
        #print ("Since we are only interested in approaching the target from above, we limited the range of theta from %f to 180 degrees"%(c2*180/np.pi))
        theta=np.arange(c2,np.pi,0.0001)
        
    else:
        #print ("Since we are only interested in approaching the target from above, we limited the range of theta from %f to 180 degrees"%(c1*180/np.pi))
        theta=np.arange(c1,c2,0.001)
    
    #We give values to the the arch which are posible solutions for the wirst. 
    x_=px+l4*np.cos(theta)
    y_=py+l4*np.sin(theta)

    #Selection of the wrist position as the mid point of the arch.       
    index=int(theta.size/2)
    theta_selected=theta[index]
    
    wx=px+l4*np.cos(theta_selected)
    wy=py+l4*np.sin(theta_selected)
    
    return(wx,wy)
    

def checkTargetInWorkspace(wx,wy):
    '''This function checks if the target position can be reached by the robotic arm Inputs are the target (x,y,z) and it retunrs False if the position cannot be reached and True if it can be
    reached'''
    
    #print ("Restriction: (l2+l3)>=sqrt(wx^2+wy^2)")
    #print (l2+l3,">=",np.sqrt(wx*wx+wy*wy),"\n")

    #Check if the wirst position is in the work-space of the robot 
    if (l2+l3)<(np.sqrt(wx*wx+wy*wy)):
        print ("The wirst is out of the work-space")
        return (False)
    
    else:   
        return (True)
    
def ComputationQ3(wx,wy):
    '''This function calculates the value of q3'''
        #Computation of the cos(q2)(c3)
    c3=(wy*wy+wx*wx-l2*l2-l3*l3)/(2*l2*l3)
    #print ("cos(q3)=",c3)
    #print ("If cos has a positivesign, the angle should be on the first or fourth quadrant")
    #print ("If cos has a negative sign, the angle should be on the second or third quadrant\n")
    
    #Computation of the sin(q3)(s3)
    #The sign choosen will determine the up or down elbow configuration
    s3=-np.sqrt(1-c3*c3)
    #Computation of q2    
    q3=np.arctan2(s3,c3)

    #if (q3*180/np.pi)>0 : 
    #    print ("q3[0]=",q3*180/np.pi,"First quadrant","or, which is equivalent:")
    #    print ("q3[1]=",360-q3*180/np.pi, "fourth quadrant")
    
    #else: 
    #    print ("q3[0]=",q3*180/np.pi+360,"fourth quadrant","or, which is equivalent:")
    #    print ("q3[1]=",-q3*180/np.pi,"first quadrant ")
        
    q3_bis=-q3
    return (q3,q3_bis)


def ComputationQ2 (wx,wy):

    c3=(wy*wy+wx*wx-l2*l2-l3*l3)/(2*l2*l3)
    #print ("We are going to consider q3=%f"%(q3*180/np.pi))
    #Computation of the cos(q2)(c2)
    s3=-np.sqrt(1-c3*c3)

    c2=(l3*s3*wy+wx*(l2+l3*c3))/(l2*l2+2*l2*l3*c3+l3*l3)
    
    #print ("cos(q2)=",c2)
    
    #Computation of the sin(q2)(s2)
    #print ("The sign of the sin of q2 could be positive or negative:")
    s2=np.sqrt(1-c2*c2)
    s2_bis=-np.sqrt(1-c2*c2)
    #print (s2,"or",s2_bis)
    
    #Computation of q2
    q2=np.arctan2(s2,c2)
    q2_bis=np.arctan2(s2_bis,c2)
    #To print the angle, uncomment the following line: 
    #print ("So we have two angles sets:")
    #print ("q2=",q2*180/np.pi)
    #print ("q3=",q3*180/np.pi)
    #print ("---")
    #print ("q2=",q2_bis*180/np.pi)
    #print ("q3=",q3*180/np.pi)

    #print ("******************************************")

    #print ("We are going to consider q3=%f"%(q3_bis*180/np.pi))
    #Computation of the cos(q2)(c2)
    q3=np.arctan2(s3,c3)
    q3_bis=-q3

    c2=(l3*(np.sin(q3_bis))*wy+wx*(l2+l3*c3))/(l2*l2+2*l2*l3*c3+l3*l3)
    
    #print ("cos(q2)=",c2)
    
    #Computation of the sin(q2)(s2)
    #print ("Here again, the sign of the sin of q2 could be positive or negative:")
    s2_1=np.sqrt(1-c2*c2)
    s2_1_bis=-np.sqrt(1-c2*c2)
    #Computation of q2
    q2_1=np.arctan2(s2_1,c2)
    q2_1_bis=np.arctan2(s2_1_bis,c2)

    #print ("So we have two angles sets:")
    #print ("q2=",q2_1*180/np.pi)
    #print ("q3=",q3_bis*180/np.pi)
    #print ("---")
    #print ("q2=",q2_1_bis*180/np.pi)
    #print ("q3=",q3_bis*180/np.pi)
    return(q2,q2_bis,q2_1,q2_1_bis)

def SelectionAngleSet(wx,wy,q2,q3):
    #print ("We are going to discard the sets that are not reaching the wrist position")
    #print ("wrist=",(wx,wy))

    #print ("So we have the following angles sets:")
    #print ("q2=",q2*180/np.pi)
    #print ("q3=",q3*180/np.pi)
    #print ("---")
    #print ("q2=",q2_bis*180/np.pi)
    #print ("q3=",q3*180/np.pi)
    #print ("---")
    #print ("q2=",q2_1*180/np.pi)
    #print ("q3=",q3_bis*180/np.pi)
    #print ("---")
    #print ("q2=",q2_1_bis*180/np.pi)
    #print ("q3=",q3_bis*180/np.pi)

    #print ("To see which of these are reaching the wirst position, we apply direct kinematics")
    #print ("Direct kinematics for a two links arm:")
    #print ("wx=l2*cos(q2)+l3*cos(q2+q3)")
    #print ("wy=l2*sin(q2)+l3*sin(q2+q3)")

    #Build arrays for each set 
    #q2_set=(q2,q2_bis,q2_1,q2_1_bis)
    q2_set=(q2[0],q2[1],q2[2],q2[3])
    #q3_set=(q3,q3,q3_bis,q3_bis)
    q3_set=(q3[0],q3[0],q3[1],q3[1])

    #for loop to test the direct kinematics
    k=0
    #We allocate two positions since only two valid configurations are possible (elbow up/down)
    q2=np.zeros(2)
    q3=np.zeros(2)
    for i in range (len(q2_set)):
        #print ("%d set of angles:"%i)
        #print (q2_set[i],q3_set[i])
        wx_reached=l2*np.cos(q2_set[i])+l3*np.cos(q2_set[i]+q3_set[i])
        wy_reached=l2*np.sin(q2_set[i])+l3*np.sin(q2_set[i]+q3_set[i])
        #print ("(wx,wy)=",(wx_reached,wy_reached))
        if ((np.abs(wx_reached-wx)<0.001) and (np.abs(wy_reached-wy)<0.001)):
            #print ("Selected set")
            q2[k]=q2_set[i]
            q3[k]=q3_set[i]
            k=k+1
    return (q2,q3)

def ComputationQ4(wx,wy,px,py,q2,q3):
    
    phi=np.arctan2((py-wy),(px-wx))

    q4=np.zeros(2)
    #To print the angle, uncomment the following line: 
    #print ("phi=",phi*180/np.pi)

    #Computation of q4
    q4[0]=phi-(q3[0]+q2[0])
    #print ("q4[0]=",q4[0]*180/np.pi)

    q4[1]=phi-(q3[1]+q2[1])
    #print ("q4[1]=",q4[1]*180/np.pi)
    
    return(q4)

def ComputationQ1(py,pz):
    q1=np.zeros(2)
    q1[0]=np.arctan2(pz,py) #Me quedan dudas de si este angulo debe invertirse. 
    q1[1]=-q1[0]
    return (q1)
    

def CheckForwardKinematics(q1,q2,q3,q4):
    a1=0.5
    a2=30.0
    a3=35.0
    a4=25.1

    d1=38.5 #NOT SURE ABOUT THIS VALUE

    x= a4*np.cos(q1)*np.cos(q2+q3+q4)+a3*np.cos(q1)*np.cos(q2+q3)+a2*np.cos(q1)*np.cos(q2)+ a1*np.cos(q1)
    y= a4*np.sin(q1)*np.cos(q2+q3+q4)+a3*np.sin(q1)*np.cos(q2+q3)+a2*np.sin(q1)*np.cos(q2)+a1*np.sin(q1)
    z= -a4*np.sin(q2+q3+q4)-a3*np.sin(q2+q3)-a2*np.sin(q2)+d1
    
    return(x,y,z)