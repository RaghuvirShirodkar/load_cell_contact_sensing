#!/usr/bin/python
import rospy
import numpy as np
import time
from std_msgs.msg import UInt32MultiArray, Float64
from geometry_msgs.msg import Twist
from force_analysis import ForceAnalysis

class LoadCellIntentController:

    def __init__(self):

        self.msg_received = False
        self.cmd_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.intent_pub_ = rospy.Publisher('/intent', Float64, queue_size=1)
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
        cmd_pub_object = Twist()
        intent_pub_object = Float64()

        while time.time() - start < 1.:
            intent_pub_object.data = 40000
            self.intent_pub_.publish(intent_pub_object)
            #cmd_pub_object.linear.x = x
            #cmd_pub_object.linear.y = y
            #cmd_pub_object.angular.z = angle_
            #self.cmd_pub_.publish(cmd_pub_object)

        intent_pub_object = Float64()
        self.intent_pub_.publish(intent_pub_object)
        #cmd_pub_object = Twist()
        #self.cmd_pub_.publish(cmd_pub_object)

if __name__ == '__main__':

    rospy.init_node('load_cell_intent_controller')
    test = LoadCellIntentController()
    while not rospy.is_shutdown():
        test.identify_load_and_publish_velocity()
    rospy.spin()
