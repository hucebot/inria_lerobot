# inria_lerobot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/downloads/)

Custom package based on [lerobot](https://github.com/huggingface/lerobot). The package is used to control the arm [So100](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md), but we implement our own pipeline to trian the policy.

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

## Record Dataset
TODO

## Trainning
TODO

## Evaluation
TODO