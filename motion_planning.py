import math
import random

import numpy as np
import time
from multi_drone import MultiDrone


# Node records the configuration and its father node
class Node:
    def __init__(self, configuration, father=None):
        self.configuration = configuration
        self.father = father
def rrt_motion_planning(sim):
    start_time = time.time()
    # initialize the setting
    initial_configuration = np.array(sim.initial_configuration, dtype=np.float32)
    goal_positions = sim.goal_positions

    # build a list to store all nodes
    tree = [Node(initial_configuration)]
    m = len(initial_configuration)

    # the bounding environment
    low = [0,0,0]
    high = [50,50,50]

    iter = 0
    max_iterations = 5000
    D = [2, 1.5, 1, 0.5]
    path = []
    while iter < max_iterations:
        sample_config = None
        if random.random() < 0.2:
            sample_config = np.array(goal_positions, dtype=np.float32)
        else:
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
        for d in D:
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
                end_time = time.time()
                return list(reversed(path)), end_time - start_time
        iter += 1
    end_time = time.time()
    return None, end_time - start_time
def run_rrt(environment_file, num_drones):
    sim = MultiDrone(num_drones=num_drones, environment_file=environment_file)
    path, time = rrt_motion_planning(sim)
    if path is not None:
        print("Path found")
    else:
        print("Path not found")
    return path, time, path is not None


