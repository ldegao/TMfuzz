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

### 1. Install CARLA
### 2. Install Autoware-Ai
### 3. Install Carla-autoware