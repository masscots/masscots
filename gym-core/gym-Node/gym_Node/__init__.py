from gym.envs.registration import register

register(
    id='Node-v0',
    entry_point='gym_Node.envs:NodeEnv',
)
