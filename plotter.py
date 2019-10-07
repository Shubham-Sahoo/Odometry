import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from array import *
gyro_data = [[0 for x in range(1)] for y in range(50000)]
#gyro_data[1][50000]
i=0
bn=1

def callback(data):
    global i
    gyro_data[i]= data.angular.z-bn
    i=i+1
    
def callback2(msg):
    global bn
    bn=msg.data


def plot():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plot', anonymous=True)

    rospy.Subscriber("enc_left", Twist , callback)
    rospy.Subscriber("bias_lz", Float32 , callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    plt.subplot(121)
    plt.hist(gyro_data, bins='auto')
    plt.title('Original data')
    plt.show()

if __name__ == '__main__':
    plot()