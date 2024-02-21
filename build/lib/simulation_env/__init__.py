from gym.envs.registration import (
     registry,
     register
 )

register(
    id="Simulation-v0",
    entry_point="simulation_env.envs:SimulationEnv",
)

