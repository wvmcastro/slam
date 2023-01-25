#!/usr/bin/env zsh

n=$1
simulation_config=$2
world_file=$3

for s in $(seq 0 $((n-1)))
do
  nohup roslaunch gazebo_ros empty_world.launch world_name:=$3 gui:=False use_sim_time:=True &
  
  sleep 10
 
  ./simulation.py $simulation_config

  killall -9 gzserver
  killall -9 roscore
  killall -9 rosmaster

  rosclean purge -y

  echo Simulation $s finished

  sleep 60
done

