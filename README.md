# Manipulation Template for IsaacLab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-0.3.1-silver)](https://isaac-sim.github.io/IsaacLab/)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

This repository is based on the old version (orbit) of the orbit extension repository from isaaclab and provides a template for research on robot manipulation tasks. It is independent of isaaclab and allows for customization of the details of manipulation tasks.



## Setup



### Dependencies

This template depends on Isaac Sim and IsaacLab. 

- [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)

It is recommended to install in a virtual environment. IsaacLab provides a default virtual environment that can be activated directly.
```bash
# Activate conda environment
conda activate isaaclab
```
### Download
```bash
git clone https://github.com/NathanWu7/isaacLab.manipulation.git
cd isaacLab.manipulation
python -m pip install -e .
```



### RL Algorithm

Install [RSL_RL](https://github.com/leggedrobotics/rsl_rl) outside of the isaacLab repository, e.g. `home/code/rsl_rl`.

```bash
git clone https://github.com/leggedrobotics/rsl_rl.git
cd rsl_rl
python -m pip install -e .
```


## Usage


```bash
#root of the package
cd isaacLab.manipulation
```
### Train a policy.
1.RobotArm
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Reach-Franka-v0 --num_envs 4096 --headless
```
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Reach-UR10-v0 --num_envs 4096 --headless
```
2.Dextrous Hand
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Repose-Cube-Allegro-v0 --num_envs 4096 --headless
```


### Play the trained policy.

1.RobotArm
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Reach-Franka-Play-v0 --num_envs 16
```
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Reach-Franka-Play-v0 --num_envs 16
```
2.Dextrous Hand
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Repose-Cube-Allegro-Play-v0 --num_envs 16
```

## Author
**Author: Qiwei Wu<br />
Email: nathan.wuqw@gmail.com**





