import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import roslib #; roslib.load_manifest('scorbot_joint')

def joint_states_callback(data):
    global current_position 
    global printJointStates
    current_position = data
    #if printJointStates:
    print (current_position.position)



def mover():
    pub_base = rospy.Publisher('/scorbot/base_position_controller/command', Float64, queue_size=10)
    pub_shoulder = rospy.Publisher('/scorbot/shoulder_position_controller/command', Float64, queue_size=10)
    pub_elbow = rospy.Publisher('/scorbot/elbow_position_controller/command', Float64, queue_size=10)
    pub_pitch = rospy.Publisher('/scorbot/pitch_position_controller/command', Float64, queue_size=10)
    pub_roll = rospy.Publisher('/scorbot/roll_position_controller/command', Float64, queue_size=10)

    # pub = rospy.Publisher('/scorbot/trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.init_node('simple_mover_scorbot')
    rate = rospy.Rate(0.2)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time

        # pub_base.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        # pub_shoulder.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        # pub_elbow.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        # pub_pitch.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        # pub_roll.publish (math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        
        pub_shoulder.publish (0)
        rate.sleep()
        
        pub_shoulder.publish (-90)
        

        rate.sleep()

if __name__ == '__main__':
    rospy.Subscriber('/scorbot/joint_states', JointState, joint_states_callback)

    try:
        mover()
    except rospy.ROSInterruptException:
        pass
