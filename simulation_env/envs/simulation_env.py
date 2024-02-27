import gym
import numpy as np
from gym import spaces
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import rospy
import time
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist

class SimulationEnv(gym.Env):
    metadata = {
        "render.modes": ["human", "rgb_array"],
    }
    def camera_cb(self, camera_img_msg):
        self.image = self.bridge.imgmsg_to_cv2(camera_img_msg, "bgr8")
    def compute_reward(self):
        self.reward = 1
        return self.reward
    def _get_observation(self):
        rospy.Subscriber("/miniature_robot/camera/image_raw", Image, self.camera_cb)
        time.sleep(0.1)
        self.image_array = np.asarray(self.image)
        #cv2.imshow("Output", self.image)
        #cv2.waitKey(1000)
        #cv2.destroyAllWindows()
        #plt.imshow(self.image_array, interpolation='nearest')
        #plt.show()
        return self.image_array
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("simulation_env")
        self.image_initial = rospy.wait_for_message("/miniature_robot/camera/image_raw", Image, timeout=2)
        self.image_initial_cv2 = self.bridge.imgmsg_to_cv2(self.image_initial, "bgr8")
        self.image_array = np.asarray(self.image_initial_cv2)
        print (self.image_array.shape)
        print (self.image_array)
        self.steering_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.observation_space = spaces.Box(low=0, high=255, shape=self.image_array.shape, dtype=np.uint8)
        self.action_space = spaces.Discrete(10)
    def reset(self):
        self.current_observation = self._get_observation()
        return self.current_observation
    def render(self, mode=None):
        pass
    def step(self, action):
        take_action = action
        steering_angle = 0
        if (take_action == 0):
            steering_angle = 0
        elif (take_action == 1):
            steering_angle = 0.10
        elif (take_action == 2):
            steering_angle = 0.15
        elif (take_action == 3):
            steering_angle = 0.20
        elif (take_action == 4):
            steering_angle = 0.30
        elif (take_action == 5):
            steering_angle = -0.10
        elif (take_action == 6):
            steering_angle = -0.15
        elif (take_action == 7):
            steering_angle = -0.20
        elif (take_action == 8):
            steering_angle = -0.25
        elif (take_action == 9):
            steering_angle = -0.30
        steering_msg = Twist()
        steering_msg.linear.x = -0.30
        steering_msg.angular.z = steering_angle
        self.steering_publisher.publish(steering_msg)
        print (take_action)
        self.current_observation = self._get_observation()
        self.reward = self.compute_reward()
        info = 0
        _ = 0
        return self.current_observation, self.reward, info, _
    def close(self):
        pass
