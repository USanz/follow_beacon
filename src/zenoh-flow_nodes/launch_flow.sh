#!/bin/bash

function ctrl_c() {
        echo "CTRL-C: destroying"
        zfctl destroy $id
        exit
}


if (($# != 1))
then
    echo "USAGE: ./launch_flow.sh file"
    exit
fi

id=$(ROS_LOCALHOST_ONLY=1 zfctl launch $1)

echo "Blocking zfctl launch started..."
echo "Press CTRL+C to destroy "

while true; do
    sleep 0.5
    trap ctrl_c INT
done