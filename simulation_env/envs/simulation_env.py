import gym
import numpy as np
from gym import spaces

class SimulationEnv(gym.Env):
    metadata = {
        "render.modes": ["human", "rgb_array"],
    }

