from BioAIR.Drones.Node_gym import Node
import os

if __name__ == "__main__":
    # generate Node object
    env = Node(0, os.getcwd() + '/config_drone2.ini', 'simple_test4')

    # Each drone has their own node object and run.
    # node.run(0)
    
    for i in range(10):
        env.reset()
        
        for t in range(100):
            env.render()
            action = env.action_space.maybe_right()
            observation, reward, done, info = env.step(action)
            print("position:",observation)
            print("reward:", reward)
            

            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break

    env.close()
