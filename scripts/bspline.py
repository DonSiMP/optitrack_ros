#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

def ChaikinSubdivisionClosedPolygon(P, Q):
    N = len(P)
    for i in range(7):
        for j in range(N):
    		Q[2*i][0] = 3./4. * P[i][0] + 1./4. * P[i+1][0];
    		Q[2*i][1] = 3./4. * P[i][1] + 1./4. * P[i+1][1];
    		Q[2*i+1][0] = 1./4. * P[i][0] + 3./4. * P[i+1][0];
    		Q[2*i+1][1] = 1./4. * P[i][1] + 3./4. * P[i+1][1];





def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


    P = np.array([[0, 0], [0, 2], [2, 3], [4, 0], [6, 3], [8, 2], [8, 0]])
    Q = [][]
    x = points[:,0]
    y = points[:,1]

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sorted_markers", sensor_msgs::PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
