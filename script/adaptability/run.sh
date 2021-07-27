#!/usr/bin/bash

echo "Starting Scalability Experiments"

launch(){
    echo "Reseting Gazebo World"
    rosservice call /gazebo/reset_world "{}"
    sleep 2
    echo "Done"
    echo "Starting controllers"
    xterm -e timeout 902 rosrun 2021_iros_grf_colletive_transport grf_rl_transport_node &
    xterm -e timeout 902 rqt_plot -t /object_state &
    echo "Starting recorder"
    timeout 900 rosbag record /object_state -O $1
    python ../bag2npy.py $1.bag &
    play ~/Music/finish.wav
}

for i in {$1..$2..1}; do
    exp=`printf "exp_%0.3d\n" $i;`
    echo "Running experiment: "$exp;
    launch $exp;
    echo "Done";
done