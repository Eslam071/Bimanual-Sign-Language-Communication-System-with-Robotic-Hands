import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import numpy as np

little = [10,20,30]
ring =   [14,23,34]
middle = [12,24,36]
index =  [13.5,23.5,33.5]
thump =  [12.5,14,35]

sum_little = 60
sum_ring = 71
sum_middle = 72
sum_index = 70.5
sum_thump = 61.5


class MyNode:
    def __init__(self):
        rospy.init_node('joint_angle_subscriber', anonymous=True)
        rospy.Subscriber('/moving', JointState, self.my_callback)
        self.pub = rospy.Publisher('movement', Int16MultiArray, queue_size=10)
        self.hand = [0, 0, 0, 0, 0, 0]        #[wrist, little, ring, index, thump]
        self.wrist = 0
        self.little = [0,0,0]
        self.ring = [0,0,0]
        self.middle = [0,0,0]
        self.index = [0,0,0]
        self.thump = [0,0,0]

    def my_callback(self, msg):
        self.wrist = msg.position[0]      #Joints_NO: 1
        self.thump = [msg.position[2] ,msg.position[3] ,msg.position[4]]   #Joints_NO: 3,8,9
        self.little = [msg.position[6] ,msg.position[7] ,msg.position[8]]     #Joints_NO: 10,11,12
        self.ring = [msg.position[10] ,msg.position[11] ,msg.position[12]]       #Joints_NO: 13,14,15
        self.middle = [msg.position[13] ,msg.position[14] ,msg.position[15]]    #Joints_NO: 6,16,17
        self.index = [msg.position[16] ,msg.position[17] ,msg.position[18]]    #Joints_NO: 7,18,19

        self.hand[0] = int(self.wrist * (110/90) )
        self.hand[1] = int( (sum_little - ((little[2]*np.cos(self.little[2])) + (little[1]*np.cos(self.little[1])) + (little[0]*np.cos(self.little[0])))) * (180/(np.pi*20)) * (75/180) * 1.8 )
        self.hand[2] = int( (sum_ring - ((ring[2]*np.cos(self.ring[2])) + (ring[1]*np.cos(self.ring[1])) + (ring[0]*np.cos(self.ring[0])))) * (180/(np.pi*20)) * (85/180) )
        self.hand[3] = int( (sum_middle - ((middle[2]*np.cos(self.middle[2])) + (middle[1]*np.cos(self.middle[1])) + (middle[0]*np.cos(self.middle[0])))) * (180/(np.pi*20)) * (90/180) * 1.5 )
        self.hand[4] = int( (sum_index - ((index[2]*np.cos(self.index[2])) + (index[1]*np.cos(self.index[1])) + (index[0]*np.cos(self.index[0])))) * (180/(np.pi*20)) * (90/180) )
        self.hand[5] = int( (sum_thump - ((thump[2]*np.cos(self.thump[2])) + (thump[1]*np.cos(self.thump[1])) + (thump[0]*np.cos(self.thump[0])))) * (180/(np.pi*20)) * (90/180) )
        # rospy.loginfo("Received joint angles: %s", msg.position)

    def publisher(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            int16data = Int16MultiArray(data=self.hand)  # setting the data attribute
            rospy.loginfo("Publishing: {}".format(int16data.data))
            self.pub.publish(int16data)
            rate.sleep()

if __name__ == '__main__':
    my_node = MyNode()
    try:
        my_node.publisher()
    except rospy.ROSInterruptException:
        pass

"""
Joints_NO:
  0- joint_1
  1- joint_2
  2- joint_3
  3- joint_8
  4- joint_9
  5- joint_4
  6- joint_10
  7- joint_11
  8- joint_12
  9- joint_5
  10- joint_13
  11- joint_14
  12- joint_15
  13- joint_6
  14- joint_16
  15- joint_17
  16- joint_7
  17- joint_18
  18- joint_19
"""