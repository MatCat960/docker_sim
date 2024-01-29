#!/usr/bin/python

import sys
import subprocess
import time

type = ['gaussian']
size = [50, 100, 200]
sensing_range = [50, 100, 500]

iterations = 10

def modify_number_of_drones(number):
    print('do sometinh here')


def main(argv):
    print("*********** STARTING SIMULATION ***********")
    print('')

    for t in type:
        for s in size:
            for r in sensing_range:
                print(f"")
                print(f"STARTING SIMULATION TYPE {t} FOR AREA: {s} and RANGE {r}")
                print(f"")

                for i in range(iterations):
                    print(f"")
                    print(f"  -- Iteration number {i}")
                    print(f"")

                    roscore = subprocess.Popen(['roscore'])
                    time.sleep(2)

                    flightmare = subprocess.Popen(['roslaunch','swarmros', 'flightmare_coverage_8.launch'])
                    time.sleep(20)
                    main = subprocess.Popen(['roslaunch','swarmros', 'flightmare_main_8.launch', f'type:={t}', f'area_size:={s}', f'robot_range:={r}', f'iteration:={i}'])

                    while(main.poll() is None):
                        continue

                    roscore.send_signal(subprocess.signal.SIGINT)
                    flightmare.send_signal(subprocess.signal.SIGINT)
                    main.send_signal(subprocess.signal.SIGINT)

                    time.sleep(5)


    

if __name__ == '__main__':
    main(sys.argv[1:])
