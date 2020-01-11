#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import UInt32MultiArray

class MovingAverage:

    def __init__(self, averaging_type='simple', window_size=1, alpha=0):
        '''
            Initialise object of MovingAverage class.
            :param:
                averaging_type: implementation of MovingAverage
                window_size: param in case of simple moving average
                alpha: param in case of exponential moving average
        '''
        rospy.loginfo("Started {} moving_average node with window size {}".format(averaging_type, window_size))
        self.base_history = []
        self.torso_history = []
        self.averaging_type = averaging_type
        self.window_size = window_size
        self.alpha = alpha
        self.pub = rospy.Publisher('/load_cell/filtered_readings', UInt32MultiArray, queue_size=1)
        rospy.Subscriber('/load_cell', UInt32MultiArray, self.callback)

    def callback(self, msg):
        '''
            Callback function executed every time a new
            message is received.
            :param:
                msg: message received through subscriber
        '''
        base_load_cells_readings = np.array(list(msg.data)[:8])
        torso_load_cells_readings = np.array(list(msg.data)[8:])
        self.base_history.append(base_load_cells_readings)
        self.torso_history.append(torso_load_cells_readings)
        pub_object = UInt32MultiArray()

        if self.averaging_type == 'simple':
            '''
                Implementation of simple moving average.
                Based on wikipedia formulation.
            '''
            if len(self.base_history) > self.window_size:
                base_history = self.base_history[-self.window_size:]
                torso_history = self.torso_history[-self.window_size:]
                base_values = np.sum(np.array(base_history), axis=0) / len(base_history)
                torso_values = np.sum(np.array(torso_history), axis=0) / len(torso_history)
                pub_object.data = np.concatenate((base_values, torso_values), axis=0)
                self.pub.publish(pub_object)

        elif self.averaging_type == 'exponential':
            '''
                Implementation of exponential moving average.
                Based on wikipedia formulation.
            '''
            base_history = np.array(self.base_history[-1:]).reshape((len(self.base_history[0]),))
            torso_history = np.array(self.torso_history[-1:]).reshape((len(self.torso_history[0]),))

            if len(self.base_history) == 1:
                base_values, torso_values = base_history, torso_history
                self.ema_base, self.ema_torso = base_values, torso_values
                pub_object.data = np.concatenate((base_values, torso_values), axis=0)
                self.pub.publish(pub_object)

            base_values = self.get_updated_ema(base_history, self.ema_base)
            torso_values = self.get_updated_ema(torso_history, self.ema_torso)
            self.ema_base, self.ema_torso = base_values, torso_values
            pub_object.data = np.concatenate((base_values, torso_values), axis=0)
            self.pub.publish(pub_object)


    def get_updated_ema(self, value, ema_value):
        '''
            Compute the updated value of the exponential
            moving average.
            :param:
                value: value at timestep (t)
                ema_value: value at timestep (t-1)
            :returns:
                new_value: updated value at timestep (t)
        '''
        update_current = map(lambda x: x * self.alpha, value)
        update_history = map(lambda x: x * (1-self.alpha), ema_value)
        new_value = np.array([sum(x) for x in zip(update_current, update_history)]).reshape((len(update_current),))
        return new_value
        

if __name__ == '__main__':

    n = rospy.init_node('moving_average')
    test = MovingAverage(averaging_type='exponential', alpha=0.125)
    rospy.spin()
