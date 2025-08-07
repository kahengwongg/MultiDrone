from motion_planning import run_rrt

def main_run():
    times = 100
    avg_time = 0
    success = 0
    for _ in range(times):
        _, time, found = run_rrt(num_drones=1, environment_file="environment_08.yaml")
        if found: avg_time += time
        success += found
    print("Average time: " + str(avg_time / times))
    print("Success rate: " + str(success / times))

if __name__ == "__main__":
    main_run()
