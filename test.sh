#!/bin/bash

t=7200  
start_time=$(date +%s)  

while true; do
    ./init.sh
    sleep 1
    current_time=$(date +%s)
    total_duration=$((current_time - start_time))
    if [ $total_duration -ge $t ]; then
        echo "Total duration exceeded $t seconds. Exiting..."
        break
    fi
    ./fuzzer.py --sim-port 4000 -t autoware --debug --density 1 --town 3 --no-traffic-lights
    status=$(docker inspect -f '{{.State.Status}}' carla-$USER)
    if [[ $status != "running" ]]; then
        echo "carla-$USER is not in 'running' state. Restarting..."
    fi
done
#  --no-lane-check 

