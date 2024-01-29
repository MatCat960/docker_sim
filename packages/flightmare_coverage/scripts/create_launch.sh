#!/bin/bash
#
# Script used to generate a launch file allows the selection of the number of drones
# and the distance between the various elements. Allows the creation of an environment
# of equidistant drones from each other, the drones are positioned starting from the origin
#

# folder name
fname=${PWD##*/} 
if [ "$fname" != "scripts" ]; then
  echo " You have to run this script form the script folder."
  exit
fi
# check that the number of drones is passed as arguments
if [ $# -eq 0 ]
  then
    echo "No arguments supplied. USAGE ./create_lauch.sh [NUMBEROFDRONES | DISTANCE]"
    exit
fi

if [ $# -eq 1 ]
  then
    echo "Only one argument supplied. USAGE ./create_lauch.sh [NUMBEROFDRONES | DISTANCE]"
    exit
fi

# ARGS
DRONESNUMBER=$1
DISTANCE=$2
RENDER=false
# GAZEBO=false

# variables
DESTINATIONFOLDER="../launch/"
LAUNCHFILENAME="flightmare_coverage"
SQRT=$(bc <<< "scale=1; sqrt($DRONESNUMBER)")
SQRT_ROUND=$(bc <<< "scale=0; sqrt($DRONESNUMBER)")
IS_SQUARE=$(bc <<< "scale=0; $SQRT_ROUND  / $SQRT") #"$SQRT_ROUND  / $SQRT " | bc -l

COLUMN=$(($SQRT_ROUND - 1))
ROW=$SQRT_ROUND

if [ 0 -eq $IS_SQUARE ]  
then
  COLUMN=$SQRT_ROUND
  ROW=$SQRT_ROUND
fi

declare -a POS_X
declare -a POS_Y

#prepare position of drones
for row in $(seq 0 $ROW); do
  for col in $(seq 0 $COLUMN); do
    POS_X+=($(($row * $DISTANCE + 1)))
    POS_Y+=($(($col * $DISTANCE + 1)))
  done
done

# Preprare the lauch file, start writing the posisition of the different elements
rm -f *.launch
touch middle_part
echo "<!-- ================================================  Spawn the quadrotor ================================================  -->" >> middle_part
for i in $(seq 1 $DRONESNUMBER); do
    POS=$(($i - 1))
    cp uav_base uav_base_tmp
    sed -i -e "s/__UAVNUMBER__/$i/" uav_base_tmp
    sed -i -e "s/__UAVX__/${POS_X[$POS]}/" uav_base_tmp
    sed -i -e "s/__UAVY__/${POS_Y[$POS]}/" uav_base_tmp
    cat uav_base_tmp >> middle_part
done
rm uav_base_tmp

for i in $(seq 1 $DRONESNUMBER); do
echo "  <!-- ================================================  RPG stuff $i ================================================  -->
" >> middle_part
    cp rotors_base rotors_base_tmp
    sed -i -e "s/__UAVNUMBER__/$i/" rotors_base_tmp
    cat rotors_base_tmp >> middle_part
done
rm rotors_base_tmp


echo "  <!-- ================================================  Flightmare Render ================================================  -->
" >> middle_part
if [ "$RENDER" = true ] ; then
  echo "<!-- RPG Flightmare Unity Render.-->
  <node pkg=\"flightrender\" type=\"RPG_Flightmare.x86_64\" name=\"rpg_flightmare_render\" unless=\"\$(arg use_unity_editor)\">
  </node>" >> middle_part
fi

# Nodelet management
echo "<arg name=\"custom_config\" default=\"\" />" >> middle_part
echo "<arg name=\"standalone\" default=\"true\" />" >> middle_part
echo "<arg name=\"debug_nodelet\" default=\"false\" />/>" >> middle_part
echo "<arg name=\"configuration_path\" value=\"\$(find swarmros)/params\" />" >> middle_part
echo "<!-- define launch-prefix for debugging -->" >> middle_part
echo "<arg     if=\"\$(arg debug_nodelet)\" name=\"launch_prefix\" value=\"xterm -e gdb --args\" />" >> middle_part
echo "<arg unless=\"\$(arg debug_nodelet)\" name=\"launch_prefix\" value=\"\" />" >> middle_part
echo "<!-- switching between standalone and nodeleted nodelet -->" >> middle_part
echo "<arg     if=\"\$(arg standalone)\" name=\"nodelet\" value=\"standalone\" />" >> middle_part
echo "<arg unless=\"\$(arg standalone)\" name=\"nodelet\" value=\"load\" />" >> middle_part
echo "<arg     if=\"\$(arg standalone)\" name=\"nodelet_manager\" value=\"\" />" >> middle_part
echo "<arg unless=\"\$(arg standalone)\" name=\"nodelet_manager\" value=\"\" />" >> middle_part

echo "<node name=\"fm_coverage\" pkg=\"nodelet\" type=\"nodelet\" args=\"\$(arg nodelet) swarmros/Flightmare_Coverage \$(arg nodelet_manager)\" output=\"screen\" launch-prefix=\"\$(arg launch_prefix)\">" >> middle_part

for i in $(seq 1 $DRONESNUMBER); do
  echo "<remap from=\"swarming/state_estimate$i\" to=\"/hummingbird$i/ground_truth/odometry\" />" >> middle_part
done
echo "<param name=\"configuration_path\" value=\"\$(arg configuration_path)\" type=\"string\" />" >> middle_part
echo "<rosparam command="load" file="$(find swarmros)/params/default.yaml"/>" >> middle_part
echo "</node>" >>  middle_part

touch "$LAUNCHFILENAME.launch"

# TOP part
cat top_part >> $LAUNCHFILENAME.launch

# MIDDLE dynamic part
cat middle_part >> $LAUNCHFILENAME.launch
rm middle_part

# BOTTOM part
cat bottom_part >> $LAUNCHFILENAME.launch

# Put file in the corrent position
mv -f $LAUNCHFILENAME.launch "${DESTINATIONFOLDER}/${LAUNCHFILENAME}.launch"

echo "Launch file created. Remember to CHANGE the number of drones in the params/default.yaml file according to what was put here!"

# Creating RVIZ file
# echo "Creating Rviz file.."

# touch "default.rviz"
# cat rviz_top_part >> default.rviz
# touch "rviz_middle_part"
# for i in $(seq 1 $DRONESNUMBER); do
#     cp rviz_agent_base rviz_agent_base_tmp
#     sed -i -e "s/__UAVNUMBER__/$i/" rviz_agent_base_tmp
#     cat rviz_agent_base_tmp >> rviz_middle_part
# done

# rm rviz_agent_base_tmp
# cat rviz_middle_part >> default.rviz
# rm rviz_middle_part
# cat rviz_bottom_part >> default.rviz

# mv -f default.rviz ../rviz/default.rviz
# echo "Rviz file Created"
