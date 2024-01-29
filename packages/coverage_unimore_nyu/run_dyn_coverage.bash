#!/bin/bash






for i in {1..15}
do

    # x0=$((-15 + RANDOM % 30))
    # y0=$((-15 + RANDOM % 30))
    # x1=$((-15 + RANDOM % 30))
    # y1=$((-15 + RANDOM % 30))
    # x2=$((-15 + RANDOM % 30))
    # y2=$((-15 + RANDOM % 30))
    # x3=$((-15 + RANDOM % 30))
    # y3=$((-15 + RANDOM % 30))
    # x4=$((-15 + RANDOM % 30))
    # y4=$((-15 + RANDOM % 30))
    # x5=$((-15 + RANDOM % 30))
    # y5=$((-15 + RANDOM % 30))
    # x6=$((-15 + RANDOM % 30))
    # y6=$((-15 + RANDOM % 30))
    # x7=$((-15 + RANDOM % 30))
    # y7=$((-15 + RANDOM % 30))
    # x8=$((-15 + RANDOM % 30))
    # y8=$((-15 + RANDOM % 30))
    # x9=$((-15 + RANDOM % 30))
    # y9=$((-15 + RANDOM % 30))
    # x10=$((-15 + RANDOM % 30))
    # y10=$((-15 + RANDOM % 30))
    # x11=$((-15 + RANDOM % 30))
    # y11=$((-15 + RANDOM % 30))
    # x12=$((-15 + RANDOM % 30))
    # y12=$((-15 + RANDOM % 30))
    # x13=$((-15 + RANDOM % 30))
    # y13=$((-15 + RANDOM % 30))
    # x14=$((-15 + RANDOM % 30))
    # y14=$((-15 + RANDOM % 30))
    # ROBOTS_NUM=$((6 + RANDOM % 14))
    GAUSS_X=$((-15 + RANDOM % 30))
    GAUSS_Y=$((-15 + RANDOM % 30))
    echo $GAUSS_X $GAUSS_Y
    gnome-terminal -- /bin/bash -c 'timeout 205s roslaunch coverage_unimore_nyu distri_supervisor_global.launch'
    gnome-terminal -- /bin/bash -c 'sleep 25 && timeout 180s roslaunch coverage_unimore_nyu coverage_centralized.launch GAUSS_X:='$GAUSS_X' GAUSS_Y:='$GAUSS_Y''
    sleep 220s
done
