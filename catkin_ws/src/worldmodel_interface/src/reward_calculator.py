import tf

class RewardCalculator:

    def __init__(self, track_number=5, car_namespace='/ackermann_vehicle'):
        self.car_namespace = car_namespace
        self.track_number = track_number
        self.reward = 0

        rospy.init_node('car_tf_listener')
        listener = tf.TransformListener()
        
    
    def _loadWaypoints(self):
        pass