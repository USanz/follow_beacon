#!/bin/bash

function ctrl_c() {
        echo "CTRL-C: destroying"
        zfctl destroy $id
        python3 /home/usanz/zs_t3/t3_ws/src/follow_beacon/src/zenoh-flow_nodes/stop_robot.py
        exit
}


if (($# != 1))
then
    echo "USAGE: ./launch_flow.sh file"
    exit
fi

id=$(ROS_LOCALHOST_ONLY=1 zfctl launch $1)

echo "Blocking zfctl launch started..."

zfctl list runtimes

echo "Press CTRL+C to destroy "

while true; do
    sleep 0.5
    trap ctrl_c INT
done