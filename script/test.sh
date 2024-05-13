#!/bin/bash
cmd="./fuzzer.py  --sim-port 4000 -t $1 --density $2 --town $3"
time=$4
echo $1
start_time=$(date +%s)  
while true; do
    ./init.sh
    sleep 1
    current_time=$(date +%s)
    total_duration=$((current_time - start_time))
    if [ $total_duration -ge $time ]; then
	echo "Total duration exceeded $time seconds. Exiting..."
	break
    fi
    cd ..
    $cmd
    cd ./script
    status=$(docker inspect -f '{{.State.Status}}' carla-$USER)
    if [[ $status != "running" ]]; then
	echo "carla-$USER is not in 'running' state. Restarting..."
    fi
done
#--no-lane-check  --no-traffic-light
