from BioAIR.Drones.Node import Node
import os

if __name__ == "__main__":
    # generate Node object
    node = Node(0, os.getcwd() + '/config_drone3.ini', 'simple_test4')

    # Each drone has their own node object and run.
    node.run(0)