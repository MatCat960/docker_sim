#! /bin/bash

# Land
rosservice call /voxl6/mav_services/land  "{}" &
rosservice call /voxl4/mav_services/land  "{}" & 
rosservice call /voxl5/mav_services/land  "{}" &

sleep 5

# Motors OFF
rosservice call /voxl6/mav_services/motors "data: false" &
rosservice call /voxl4/mav_services/motors "data: false" &
rosservice call /voxl5/mav_services/motors "data: false" &
