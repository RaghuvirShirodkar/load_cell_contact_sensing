#!/usr/bin/python
import rospy
import numpy as np
import math
from std_msgs.msg import UInt32MultiArray

class ForceAnalysis:

    def __init__(self, baseline_values):

        self.reference_base_values = baseline_values[:8]
        self.reference_torso_values = baseline_values[8:]
        self.contact_threshold = 2000.
        self.distances = [28., 15., 28., 15.]
        self.load_cell_placement = [2, 1, 0, 4, 5, 6, 7, 3]
        self.msg_received = False
        rospy.Subscriber('/load_cell', UInt32MultiArray, self._callback)

    def _callback(self, msg):

        self.base_values = msg.data[:8]
        self.torso_values = msg.data[8:]
        self.msg_received = True

    def _extract_force_pairs(self):
        
        base_pairs, ref_pairs = [], []

        for i in range(0, 8, 2):
            temp_base_val, temp_ref_val = [], []
            for j in self.load_cell_placement[i:i+2]:
                temp_base_val.append(self.base_values[j])
                temp_ref_val.append(self.reference_base_values[j])
            base_pairs.append(temp_base_val)
            ref_pairs.append(temp_ref_val)
                
        return (base_pairs, ref_pairs)

    def _check_for_input(self):

        if all(np.isclose(self.base_values, self.reference_base_values, atol=self.contact_threshold)):
            return False
        return True
    
    def compute_angle(self):
        
        while not self.msg_received:
            pass
            #rospy.loginfo('No data received yet')

        while not self._check_for_input():
            pass
            #rospy.loginfo('Data received, but within contact threshold')

        base_pairs, ref_pairs = self._extract_force_pairs()
        index = 0
        for pair_x, pair_y in zip(base_pairs, ref_pairs):
            for val_x, val_y in zip(pair_x, pair_y):
                if abs(val_x - val_y) > self.contact_threshold:
                    print ('Difference: {}'.format((pair_x[0] - pair_x[1])/10.))
                    angle_ = math.atan2((pair_x[0] - pair_x[1])/10., self.distances[index])
                    self.msg_received = False
                    return (index, angle_)
            index += 1
        return (None)

if __name__ == '__main__':

    rospy.init_node('force_analysis', anonymous=True)
    reference_values = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    test = ForceAnalysis(reference_values)
    test_angle = test.compute_angle()
    print (test_angle)
    rospy.spin()
