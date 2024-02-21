import gym
import numpy as np
from gym import spaces
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import rospy
import time

class SimulationEnv(gym.Env):
    metadata = {
        "render.modes": ["human", "rgb_array"],
    }
    def camera_cb(self, camera_img_msg):
        self.image = self.bridge.imgmsg_to_cv2(camera_img_msg, "bgr8")
    def _get_observation(self):
        rospy.Subscriber("/miniature_robot/camera/image_raw", Image, self.camera_cb)
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("simulation_env")
        self._get_observation()
        time.sleep(0.1)
        self.image_array = np.asarray(self.image)
        print (self.image_array.shape)
        self.observation_space = spaces.Box(low=0, high=255, shape=self.image_array.shape, dtype=np.uint8)
    def reset(self):
        return self.observation_space
    def render(self, mode=None):
        cv2.imshow("Output", self.image)
        cv2.waitKey(1)
    def step(self):
        current_observation = self._get_observation(self)
        return current_observation
    def close():
        pass
