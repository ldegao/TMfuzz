#!/bin/bash

t=3600  
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
    ./fuzzer.py
    status=$(docker inspect -f '{{.State.Status}}' carla-$USER)
    if [[ $status != "running" ]]; then
        echo "carla-$USER is not in 'running' state. Restarting..."
    fi
done

