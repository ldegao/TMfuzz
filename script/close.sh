for pid in $(ps -u $(whoami) -o pid,command | grep "/usr/bin/python2 /opt/ros/melodic/bin/rostopic echo /decision_maker/state" | awk '{print $1}'); do
    kill $pid
done
