# Vision Enabled Miniature Skid Steer Mobile Robot

This is Reinforcement Learning Environment for Miniature Robot Simulation in Gazebo 

[Miniature Robot Gazebo Simulation](https://github.com/munn33b/miniature-robot-gazebo-simulation)

To Install this environment, clone this Repo anywhere,

```bash
git clone https://github.com/munn33b/miniature-rl-env.git
```

 then run:

```bash
cd miniature-rl-env
```

and Install this Environment

```bash
pip3 install .
```

Now you should be able to run this import this environment and run it in your Python Script.

### Simple Example

```python
import gym
import simulation_env

env = gym.make("Simulation-v0")
```

