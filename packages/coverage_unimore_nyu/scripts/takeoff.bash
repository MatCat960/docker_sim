#! /bin/bash

# Motors ON
rosservice call /voxl5/mav_services/motors "data: true"
rosservice call /voxl4/mav_services/motors "data: true"
rosservice call /voxl6/mav_services/motors "data: true"

sleep 1

# Takeoff
rosservice call /voxl6/mav_services/takeoff  "{}"
rosservice call /voxl4/mav_services/takeoff  "{}"
rosservice call /voxl5/mav_services/takeoff  "{}"

sleep 5
# Go to (Not Relative)
rosservice call /voxl6/mav_services/goTo "[-0.5, 0.0, 0.5, 0.0]"
rosservice call /voxl4/mav_services/goTo "[0.5, 0.5, 0.5, -2.35]"
rosservice call /voxl5/mav_services/goTo "[0.5, -0.5, 0.5, 2.35]"

# sleep 5
# 
# rosservice call /voxl6/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'"
# rosservice call /voxl4/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'"
# rosservice call /voxl5/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'"
