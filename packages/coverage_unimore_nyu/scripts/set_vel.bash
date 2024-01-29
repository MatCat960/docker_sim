#! /bin/bash

rosservice call /voxl6/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'" &
rosservice call /voxl4/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'" &
rosservice call /voxl5/trackers_manager/transition "tracker: 'std_trackers/VelocityTrackerAction'"
