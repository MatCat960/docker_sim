#!/bin/bash

# x0=$((-10 + RANDOM % 10))
# y0=$((-10 + RANDOM % 10))
# x1=$((-10 + RANDOM % 10))
# y1=$((-10 + RANDOM % 10))
# x2=$((-10 + RANDOM % 10))
# y2=$((-10 + RANDOM % 10))
# x3=$((-10 + RANDOM % 10))
# y3=$((-10 + RANDOM % 10))
# x4=$((-10 + RANDOM % 10))
# y4=$((-10 + RANDOM % 10))
# x5=$((-10 + RANDOM % 10))
# y5=$((-10 + RANDOM % 10))




for i in {1..20}
do
    GAUSS_X=$((-15 + RANDOM % 30))
    GAUSS_Y=$((-15 + RANDOM % 30))
    echo $GAUSS_X $GAUSS_Y
    gnome-terminal -- /bin/bash -c 'timeout 210s roslaunch coverage_unimore_nyu distri_supervisor_global.launch'
    gnome-terminal -- /bin/bash -c 'sleep 25 && timeout 180s roslaunch coverage_unimore_nyu coverage_centralized.launch GAUSS_X:='$GAUSS_X' GAUSS_Y:='$GAUSS_Y''
    sleep 230s
done
