# TM-fuzzer

An innovative open-source framework designed to improve the safety and reliability of Autonomous Vehicles (AVs) through enhanced scenario-based testing. 

Developed using insights from cutting-edge research, TM-fuzzer leverages unique techniques for traffic management and scenario diversity to identify critical scenarios that expose potential vulnerabilities in ADS.

## Key Features

**NPC Traffic Management:** TM-fuzzer introduces a novel method for Non-Player Character (NPC) traffic management, which includes dynamic insertion, deletion, and management of NPCs. This approach increases interactions between NPCs and the ego vehicle, enhancing the complexity of test scenarios.

**Diversity Analysis through Clustering:** We employ clustering analysis to ensure the diversity of test scenarios. By analyzing the features from vehicle trajectories in previous accidents, TM-fuzzer optimizes scenario selection to expose new and unique ADS defects.

**Efficient Scenario Searching:** The framework dynamically generates NPCs and utilizes advanced clustering techniques to create a diverse range of realistic traffic situations, significantly improving the detection rate of ADS vulnerabilities.

![](./img/overview.png)

## Testing environment

The following environment was used to perform the testing for this
project:

* Hardware
  * CPU: Intel(R) Xeon(R) Gold 5120 CPU @ 2.20GHz
  * GPU: Tesla V100-32GB (x2)
  * RAM: 128 GB
* OS & SW
  * Ubuntu 18.04.6 LTS with Linux 5.4.0-139-generic
    * This is strictly required by Autoware, one of our test targets
  * Python 3.6.9

In addition, CARLA (the simulator we use) requires the following:
(based on `https://carla.readthedocs.io/en/0.9.13/build_linux/`)
  * Ubuntu 18.04
  * 30GB disk space
  * An adequate GPU
  * Two TCP ports and good internet connection

## Installation

> Before installing TM-fuzzer, make sure docker and nvidia-docker2 has been installed properly in your machine.

### 1. Install CARLA 0.9.13

#### Installing 
Please refer to the official CARLA installation guide:
[Installing Carla from docker](https://carla.readthedocs.io/en/0.9.13/download/)

Or just pull by:
```
docker pull carlasim/carla:0.9.13
```

#### Quick-running Carla
Carla can be run using a wrapper script `run_carla.sh`.
If you have multiple GPUs installed, it is recommended that
you "pin" Carla simulator to one of the GPUs (other than #0).
You can do that by opening `run_carla.sh` and modifying the following:
```
-e NVIDIA_VISIBLE_DEVICES={DESIRED_GPU_ID} --gpus 'device={DESIRED_GPU_ID}
```

To run carla simulator, execute the script:
```sh
$ ./run_carla.sh
```
It will run carla simulator container, and name it carla-${USER} .

To stop the container, do:
```sh
$ docker rm -f carla-${USER}
```

### 2. Install carla-autoware docker

Please refer to the official [carla-autoware](https://github.com/carla-simulator/carla-autoware) installation guide, and in order to fit our own mechine (which works without external network access capabilities), and make it work in TM-fuzzer, we make some modifications in [our own forks](https://github.com/cfs4819/carla-autoware/tree/TMfuzz).

Our Modifications:
- Add *proxy server*, *nameserver*, *ros source* in the dockerfile, which can be deleted if you don't need them.
- Add our own `entrypoint.sh` so we can run simulation directly after the container is started.
- Add a shell script `pub_initialpose.py` so we can easily change the initial pose of the ego vehicle.
- Add a shell script `reload_autoware.sh` so we can easily reload the simulation without restarting the container.
- Add some camera in `objects.json` for recording the simulation.
- Upgraded the carla version to 0.9.13, file changed in `update_sim_code.patch`, `carla-autoware-agent/launch/carla_autoware_agent.launch`

So, first clone the carla-autoware repo modified by us:

```sh
git clone https://github.com/cfs4819/carla-autoware/tree/TMfuzz
```
then make some modifications in `Dockerfile` depends on your mechine.

Then, download the additional files

```sh
cd carla-autoware/
git clone https://bitbucket.org/carla-simulator/autoware-contents.git
```

Last, build the carla-autoware repo

```sh
./build.sh
```

### 3.Installing ROS-melodic

ROS is required on the host in order for TM-Fuzzer to communicate with
the Autoware container.

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
source /opt/ros/melodic/setup.bash
```

## Usage

### 1. Run carla simulator

First check if carla is already running:
```
docker ps | grep carla-$USER
```

If not, run Carla container by executing:
```sh
docker run --name="carla-$USER" \
    -d --rm \
    -p 2000-2002:2000-2002 \
    --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 --gpus 'device=0' \
    carlasim/carla:0.9.10.1 \
    /bin/bash -c  \
    'SDL_VIDEODRIVER=offscreen CUDA_DEVICE_ORDER=PCI_BUS_ID \
    CUDA_VISIBLE_DEVICES=1 ./CarlaUE4.sh -ResX=640 -ResY=360 \
    -nosound -windowed -opengl \
    -carla-rpc-port=2000 \
    -quality-level=Epic'
```

### 2. Prepare environment

```sh
mkdir -p /tmp/fuzzerdata
sudo chmod 777 /tmp/fuzzerdata
mkdir -p bagfiles
sudo chmod 777 $HOME/.Xauthority
source /opt/ros/melodic/setup.bash
```

### 3. Run fuzzing

* Testing Behavior Agent

```sh
cd ~/drivefuzz/src
./fuzzer.py -o out-artifact -s seed-artifact -c 5 -m 5 -t behavior --timeout 60 --town 1 --strategy all
```

* Outputs (bugs, mutated test inputs, metadata, ...) will be stored in `out-artifact`
* Sample seeds are provided in `seed-artifact`
* Adjust the number of cycles (`-c`) and size of mutated population (`-m`)
    (please refer to Algorithm 1 in the paper)
* You can try other mutation strategies (congestion, entropy, instability,
    trajectory); (please refer to Section 4.2.4 in the paper)
* Check `./fuzzer.py --help` for further instructions

Please note that Behavior Agent has a bug in the controller (that we found and
reported), which leads to frequent lane invasions and finishing before
reaching the goal. You can prevent these from triggering the misbehavior
detector by adding `--no-lane-check` and `--no-other-check` flags, e.g.,

```sh
./fuzzer.py -o out-artifact -s seed-artifact -c 5 -m 5 -t behavior --timeout 60 --town 1 --strategy all --no-lane-check --no-other-check
```

* Testing Autoware
```sh
cd ~/drivefuzz/src
rm -rf out-artifact
./fuzzer.py -o out-artifact -s seed-artifact -c 5 -m 5 -t autoware --timeout 60 --town 1 --strategy all --sim-port 2000
```


## Vioaltion details

### 1. Category A
For Category A scenarios, involving solely the ego vehicle, identified issues predominantly pertain to Autoware's interactions with the map, absent of direct interactions with other traffic entities. In such scenarios, erroneous behaviors of Autoware are not triggered by the presence or actions of external NPC. The identified violations in Category A include:

![Visual representation of Category A violations identified in Autoware, illustrating scenarios involving only the ego vehicle and the resulting errors.](./img/Violation_A.png)

- **A-1**: Inability to accurately stop at the destination, as illustrated in Figure (A-1). Autoware may miscalculate the timing of braking when approaching the predetermined destination, causing the vehicle to stop too far from the endpoint.

- **A-2**: Misinterpretation of destination on the opposite lane, as illustrated in Figure (A-2). Autoware may erroneously perceive a destination on the opposite lane as being on its current lane, leading to premature stopping.

- **A-3**: Erroneous braking on slopes causing permanent halt, as illustrated in Figure (A-3). During turns into uphill slopes, Autoware's overly cautious approach and low speed may result in a miscalculated braking action, preventing the vehicle from restarting and continuing uphill, thus resulting in a permanent halt.

- **A-4**: Misjudgment of the road surface as an obstacle on downhill slopes, as illustrated in Figure (A-4). Autoware might incorrectly interpret normal road surface changes as obstacles while descending slopes, leading to an unwarranted halt.

- **(A-5)**. Autoware may encounter difficulties in identifying the correct exit in roundabout driving scenarios. Due to the structure of roundabouts and continuous exit points, Autoware might miss the intended exit, resulting in circular driving without reaching the destination.

- **A-6**: Misjudgment of destination at intersections, as illustrated in Figure (A-6). Autoware may incorrectly ascertain the location of the destination at intersections, potentially causing the vehicle to miss essential turns and deviate from the intended route.

- **A-7**: Excessive turning angle causing lane markings, as illustrated in Figure (A-7). Autoware may execute turns with excessive angles, leading to the vehicle crossing lane boundaries.

- **A-8**: Insufficient turning radius resulting in the collision with walls, as illustrated in Figure (A-8) Autoware might perform turns with insufficient radius, causing collisions with roadside obstacles or walls.

### 2. Category B

Scenarios involving interactions between the ego vehicle and another NPC (Category B) are critical in assessing Autoware's performance in dynamic traffic environments. These scenarios include interactions with other vehicles, non-motorized vehicles, or pedestrians. The following violations were identified in Category B:

![Visual representation of Category B violations in Autoware (1), illustrating scenarios involving the ego vehicle and interactions with an NPC leading to various errors.](./img/Violation_B1.png)

- **B-1**: Delayed braking resulting in rear-end collisions, as illustrated in Figure (B-1). Autoware fails to promptly recognize the deceleration or stopping of a vehicle ahead, resulting in inadequate braking and subsequent rear-end collisions.

- **B-2**: Collision due to non-detection of non-motorized vehicles or pedestrians ahead, as illustrated in Figure (B-2). Autoware fails to correctly identify or predict the presence and movement of non-motorized vehicles or pedestrians ahead, leading to collisions due to the absence of appropriate evasive actions.

- **B-3**: Collisions with large vehicles ahead during lane changes, as illustrated in Figure (B-3). Autoware inaccurately assesses the relative position and speed of large vehicles ahead during lane-changing maneuvers, resulting in collisions.

- **B-4**: Collisions due to reckless lane changes, as illustrated in Figure (B-4). Autoware's untimely lane changes lead to collisions with vehicles in adjacent lanes.

- **B-5**: Collisions on slopes due to non-detection of vehicles ahead, as illustrated in Figure (B-5). Autoware fails to timely detect vehicles ahead while driving on slopes, causing collisions.

- **B-6**: Collisions within roundabouts during entry, as illustrated in Figure (B-6). During the process of entering a roundabout, Autoware may collide with vehicles inside the roundabout due to incorrect assessment of their dynamics or improper timing of entry.

- **B-7**: Collisions during reckless lane changes in roundabouts, as illustrated in Figure (B-7). Autoware may collide with other vehicles due to inappropriate lane changing decisions while entering roundabouts.

- **B-8**: Lane marking violations at T-intersections while avoiding merging vehicles, as illustrated in Figure (B-8). Autoware may cross lane markings at T-intersections in attempts to avoid vehicles merging into the lane, resulting in traffic violations.

- **B-9**: Collisions with vehicles in the original lane during merging, as illustrated in Figure (B-9). At T-intersections, Autoware may collide with vehicles in the original lane due to misjudgment of their position and speed while attempting to merge.

- **B-10**: Collisions with vehicles in the new lane during merging, as illustrated in Figure (B-10). At T-intersections, Autoware may collide with vehicles in the new lane due to misjudgment of traffic flow or improper timing of merging.

![Visual representation of Category B violations in Autoware (2), illustrating scenarios involving the ego vehicle and interactions with an NPC leading to various errors.](./img/Violation_B2.png)

- **B-11**: Collisions with other straight-moving vehicles, as illustrated in Figure (B-11). At crossroads, Autoware may collide with other vehicles moving in the same direction during straight-line movement.

- **B-12**: Collisions with oncoming straight-moving vehicles during unsignaled left turns, as illustrated in Figure (B-12). During unsignaled left turns at intersections, Autoware fails to correctly predict conflicts with vehicles moving straight in the new lane, leading to collisions.

- **B-13**: Collisions with oncoming straight-moving vehicles during unsignaled left turns across opposite lanes, as illustrated in Figure (B-13). At intersections, Autoware may collide with vehicles moving straight in the opposite lane while performing unsignaled left turns, resulting in conflict and collision.

- **B-14**: Collisions with vehicles making left turns from opposite corners during unsignaled left turns, as illustrated in Figure (B-14). During unsignaled left turns at intersections, Autoware may collide with vehicles executing left turns from diagonal corners.

### 3. Category C
Scenarios involving the ego vehicle and multiple NPC (Category C) represent complex traffic scenarios where Autoware engages in interactions with several traffic participants simultaneously. In these scenarios, accidents or violations arise not merely from individual module errors within Autoware but from a complex interplay of multiple intricate and unique factors. Key violations identified in Category C include:

![Visual representation of Category C violations in Autoware, illustrating scenarios involving the ego vehicle and interactions with NPCs leading to various errors.](./img/Violation_C.png)

- **C-1**: Incorrect road planning in congested traffic, as illustrated in Figure (C-1). Autoware might mistakenly deem the destination unreachable due to temporary road congestion, prompting it to re-plan its route and ultimately fail to reach the destination within the allocated time.

- **C-2**: Entering the opposing lane in traffic congestion, as illustrated in Figure (C-2). During congested traffic conditions, Autoware's attempts to overtake and circumvent congestion may inadvertently result in entering the opposing lane, thus causing a lane violation.

- **C-3**: Inability to Issue next command in complex road conditions, as illustrated in Figure (C-3). Repeated starting and stopping due to frequent overtaking in adjacent lanes may expose a flaw in Autoware's decision-making logic. Specifically, Autoware may stop issuing the next command, leading to the AV losing control and resulting in collisions.

- **C-4**: Overly conservative driving preventing exit from roundabout in complex conditions, as illustrated in Figure (C-4). Autoware's overly cautious driving strategy, especially in roundabouts, may hinder its ability to exit, especially under complex conditions, thereby causing lasting traffic congestion.

- **C-5**: Incorrect judgment of complex scenarios ahead during overtaking leading to collision, as illustrated in Figure (C-5). Autoware attempts to overtake to avoid an accident ahead. Autoware may falsely conclude that an overtaking maneuver remains incomplete due to adjacent lane vehicles, leading to unnecessary continuation and subsequent collision with the curb.

- **C-6**: Overly conservative driving preventing completion of unsignaled left turn at complex intersections, as illustrated in Figure (C-6). Similar to C-4, Autoware's overly conservative driving strategy at intersections can lead to the inability to complete unsignaled left turns in complex road conditions, causing permanent traffic congestion.

- **C-7**: Incorrect judgment in complex scenarios during unsignaled left turn leading to collision, as illustrated in Figure (C-7). Similar to C-5, Autoware attempts to evade oncoming straight-moving vehicles. However, due to vehicles in the opposing lane, Autoware incorrectly assumes the evasion maneuver is incomplete and continues, resulting in a collision with the curb.

## Citation
If you use TM-fuzzer in your research, please cite our paper:

> Shenghao Lin, Fansong Chen, Laile Xi et al. TM-fuzzer: fuzzing autonomous driving systems through traffic management, 03 April 2024, PREPRINT (Version 1) available at Research Square [https://doi.org/10.21203/rs.3.rs-4185312/v1]

BibTeX:
```
@inproceedings{Kim_2022, series={CCS ’22},
   title={DriveFuzz: Discovering Autonomous Driving Bugs through Driving Quality-Guided Fuzzing},
   url={http://dx.doi.org/10.1145/3548606.3560558},
   DOI={10.1145/3548606.3560558},
   booktitle={Proceedings of the 2022 ACM SIGSAC Conference on Computer and Communications Security},
   publisher={ACM},
   author={Kim, Seulbae and Liu, Major and Rhee, Junghwan “John” and Jeon, Yuseok and Kwon, Yonghwi and Kim, Chung Hwan},
   year={2022},
   month=nov, collection={CCS ’22} }
```