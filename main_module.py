from motion_planning import run_rrt

def main_run():
    print("Testing00")
    run_rrt(num_drones=1, environment_file="environment_00.yaml")
    print("Testing01")
    run_rrt(num_drones=2, environment_file="environment_01.yaml")
    print("Testing02")
    run_rrt(num_drones=4, environment_file="environment_02.yaml")

    print("main_function")
    times = 50
    avg_time = 0
    success = 0
    for _ in range(times):
        _, time, found = run_rrt(num_drones=1, environment_file="environment_00.yaml")
        avg_time += time
        success += found
    print("Average time: " + str(avg_time / times))
    print("Success rate: " + str(success / times))

if __name__ == "__main__":
    main_run()
