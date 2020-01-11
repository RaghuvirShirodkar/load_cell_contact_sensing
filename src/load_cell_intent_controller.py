#!/usr/bin/python
import rospy
import numpy as np
import time
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Twist
from force_analysis import ForceAnalysis

class LoadCellIntentController:

    def __init__(self):

        self.msg_received = False
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/load_cell/filtered_readings', UInt32MultiArray, self._callback)

    def _callback(self, msg):

        self.reference_values = msg.data
        self.msg_received = True

    def _get_velocity_magnitude(self, index):
        
        switcher =  {
                        0: (0., 1.),
                        1: (1., 0.),
                        2: (0., -1.),
                        3: (-1., 0.)
                    }
        return switcher.get(index, (0., 0.))

    def identify_load_and_publish_velocity(self):

        while not self.msg_received:
            pass
            #rospy.loginfo('No data received yet')

        start = time.time()
        load_ = ForceAnalysis(self.reference_values)
        index, angle_ = load_.compute_angle()
        print ('Direction: {}, Angle: {}'.format(index, angle_))
        self.msg_received = False

        x, y = self._get_velocity_magnitude(index)
        pub_object = Twist()

        while time.time() - start < 2.:
            pub_object.linear.x = x
            pub_object.linear.y = y
            pub_object.angular.z = angle_
            self.pub_.publish(pub_object)

        pub_object = Twist()
        self.pub_.publish(pub_object)

if __name__ == '__main__':

    rospy.init_node('load_cell_intent_controller')
    test = LoadCellIntentController()
    while not rospy.is_shutdown():
        test.identify_load_and_publish_velocity()
    rospy.spin()
