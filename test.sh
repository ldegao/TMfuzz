#!/bin/bash

t=86400  
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


    if [ -n "$1" ]; then
        cmd="./fuzzer.py  --sim-port 4000 -t $1 --density 0.75 --town 3"
    else
        cmd="./fuzzer.py  --sim-port 4000 -t autoware --debug --density 0.75 --town 3 --no-lane-check --no-traffic-lights"
    fi

    $cmd

    status=$(docker inspect -f '{{.State.Status}}' carla-$USER)
    if [[ $status != "running" ]]; then
        echo "carla-$USER is not in 'running' state. Restarting..."
    fi
done

