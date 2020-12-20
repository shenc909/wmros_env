import tf
import rospy
from std_msgs.msg import Float64, Bool
import numpy as np

COORD_PATH = 'gym_assets/track_coords'

class RewardPublisher:

    def __init__(self):
        self.waypoint_index = 0

        self.reward_pub = rospy.Publisher('reward', Float64, queue_size=5)
        self.done_pub = rospy.Publisher('done', Bool, queue_size=5)
        rospy.init_node('car_tf_listener')
        self.listener = tf.TransformListener()

        # Get parameters
        self.track_number = rospy.get_param('~track_number', 1)
        self.car_namespace = rospy.get_param('~car_namespace', '/ackermann_vehicle')
        self.waypoint_reward = rospy.get_param('~waypoint_reward', 10)
        self.time_reward = rospy.get_param('~time_reward', -1)
        self.threshold_distance = rospy.get_param('~threshold_distance', 1)

        # Load waypoints for track
        self._loadWaypoints()

    def getReward(self):

        if self.waypoint_index >= len(self.waypoints):
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            return
        
        position, quarternion = listener.lookupTransform(car_namespace+'/base_link', 'world', rospy.Time())
        car_coordinate = np.array([position[0], position[1]])

        target_waypoint = self.waypoints[self.waypoint_index]
        car2point_vector = target_waypoint - car_coordinate

        dist = np.linalg.norm(car2point_vector)

        if dist < self.threshold_distance:
            self.waypoint_index += 1
            reward_msg = Float64()
            reward_msg.data = self.waypoint_reward
            self.reward_pub.publish(reward_msg)


        
    
    def _loadWaypoints(self):
        self.waypoints = np.load(COORD_PATH + f'track{self.track_number}.dxf.npy')

if __name__ == "__main__":

    rp = RewardPublisher()

    try:
        while not rospy.is_shutdown():
            rp.getReward()

    except rospy.ROSInterruptException:
        pass