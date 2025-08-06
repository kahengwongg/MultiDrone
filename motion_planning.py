import math
import numpy as np
from multi_drone import MultiDrone


# Node records the configuration and its father node
class Node:
    def __init__(self, configuration, father=None):
        self.configuration = configuration
        self.father = father
def rrt_motion_planning(sim):
    # initialize the setting
    initial_configuration = sim.initial_configuration
    goal_positions = sim.goal_positions

    # build a list to store all nodes
    tree = [Node(initial_configuration)]
    m = len(initial_configuration)

    # the bounding environment
    low = [0,0,0]
    high = [50,50,50]

    iter = 0
    max_iterations = 1000
    D = 15 # pick a suitable number as d
    path = []
    while iter < max_iterations:
        sample_config = np.random.uniform(low, high, size=(m,3)).astype(dtype=np.float32)
        min_dist = math.inf
        nearest_node = None

        # find the nearest point
        for node in tree:
            dist = np.linalg.norm(sample_config - node.configuration)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        assert nearest_node is not None
        # calculate the unit d
        unit_direction = (sample_config - nearest_node.configuration)/min_dist

        # find the valid node: less than D, valid config, collision-free motion
        new_node = None
        for d in range(D, 0, -1):
            start = nearest_node.configuration
            end = start+d*unit_direction
            if sim.is_valid(end) and sim.motion_valid(start, end):
                new_node = Node(end, nearest_node)
                break

        if new_node is not None:
            tree.append(new_node)
            if sim.is_goal(new_node.configuration):
                node = new_node
                while node:
                    path.append(node.configuration)
                    node = node.father
                return path.reverse()
    return None

sim = MultiDrone(num_drones=2, environment_file="environment.yaml")
path = rrt_motion_planning(sim)

sim.visualize_paths(path)


