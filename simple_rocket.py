import gym
import time
import logging
from gym.utils import seeding
import pybullet as p
import numpy as np
import os
import copy

__all__ = ['PoolEnv']


class RocketEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'renderer': p.ER_TINY_RENDERER,  # p.ER_BULLET_HARDWARE_OPENGL
    }
    action_space = gym.spaces.Box(low=np.asarray([0, -0.8, 0]), high=np.asarray([np.pi * 2, 0.8, 1]), dtype=np.float64)
    observation_spaces = {'one_ball': gym.spaces.Box(low=-5, high=5, shape=(1, 3), dtype=np.float64)}

    def __init__(self, render=False, render_real_time=False, time_step=0.0002, stable_time=0.5,
                 draw_trajectory=False, draw_time_step=0.1, visualize_force=False, log_level=logging.WARN):

        # Connect to physics engine
        # if self._render or (self._record_video and self.metadata['renderer'] is p.ER_BULLET_HARDWARE_OPENGL):
        self._client_id = p.connect(p.GUI)
        # else:
        #     self._client_id = p.connect(p.DIRECT)
        
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(time_step)
        p.setGravity(0, 0, -9.8)

        self.rocket = p.loadURDF(
            'rocket.urdf', [0, 0, 0], globalScaling=0.001,
        )
    
    def step(self):
        p.step()

    def reset(self):
        p.resetBasePositionAndOrientation(self.rocket, [0, 0, 0], [0, 0, 0, 1])

    def fire(self):
        p.applyExternalForce(self.rocket, -1, forceObj=[0, 0, 20000], posObj=[0, 0, 0], 
        flags=p.LINK_FRAME)

env = RocketEnv()
while True:
    env.fire()
    p.stepSimulation(env._client_id)
    time.sleep(1./240.)
# while True:
#     keys = p.getKeyboardEvents()
#     if ShortCut.QUIT in keys:
#         break
#     if ShortCut.RESET in keys:
#         environment.reset()

#     if random_action:
#         action = environment.action_space.sample()
#     else:
#         action = None
#         # TODO: action from model
#     reward, observation, terminated, info = environment.step(action)
#     logger.info("Reward = %.3f", reward)

#     if terminated:
#         logger.info("Episode terminated.")
#         environment.reset()
# environment.close()