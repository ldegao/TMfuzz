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

### 1. Install CARLA 0.9.13

#### Installing 
Please refer to the official CARLA installation guide:
[Installing Carla from docker](https://carla.readthedocs.io/en/0.9.13/download/)

Or just try:
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

### 2. Install Autoware-Ai

Please refer to the official Autoware-Ai installation guide:

### 3. Install Carla-autoware

## Usage

## Vioaltion details

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