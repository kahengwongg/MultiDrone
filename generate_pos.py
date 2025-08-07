import numpy as np
def generate_positions_np(num):
    initial_positions = np.round(np.random.uniform([0,0,0], [30,30,30], size=(5,3)).astype(int)).tolist()

    goal_positions = np.round(np.random.uniform([30,30,30], [50,50,50], size=(5,3)).astype(int)).tolist()

    print("initial_configuration =\n", initial_positions)
    print("\ngoals =\n", goal_positions)

generate_positions_np(10)