from BioAIR.Drones.Node_wrapper import Node
import os
import gym
import gym_Node

if __name__ == "__main__":
    # generate Node object
    env = gym.make('Node-v0')
    
    # Each drone has their own node object and run.
    for i in range(10):
        env.reset(0, os.getcwd() + '/config_drone2.ini', 'simple_test4')
        
        for t in range(200):
            env.render()
            action = env.action_space.maybe_right()
            observation, reward, done, info = env.step(action)
            print("position:",observation)
            print("reward:", reward)
            

            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break

    env.close()
