# inria_lerobot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/downloads/)

Custom package based on [lerobot](https://github.com/huggingface/lerobot). The package is used to control the arm [So100](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md), but we implement our own pipeline to control the robot, create the dataset and to train a policy.

# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)
- LeRobot arm

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build.sh
```

To run the development container, use the following command:

```bash
sh run.sh
```

# Usage

## Teleoperation

We prepare a script to teleoeprate the robot. You can use the following command to run the script:

```bash
python script/control_robot.py --mode=teleoperation
```

It's important to note that the teleoperation it used the zero position as the one first detected. So the idea is tu put both arm in the same position and then run the script. The script will detect the zero position and use it as the reference to send the delta movement for each joint.

## Record Dataset

We provide a script to record the dataset. The script will record the dataset in the `dataset` folder with the task_name as a subfolder. The dataset will be recorded in the following format:

```bash
python scripts/control_robot.py --mode=teleoperation --record_dataset=true --dataset_task="Simple cross motion"
```

## Replay Dataset

We provide a script to replay a specific episode of the dataset. The script will replay the dataset in the `dataset` folder with the task_name as a subfolder. The dataset will be replayed in the following format:

```bash
python scripts/control_robot.py --dataset_task="Simple cross motion" --replay_episode=00001
```

## Trainning
TODO

## Evaluation
TODO