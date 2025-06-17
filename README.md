# Manipulation Template for IsaacLab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.5.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-2.0.2-silver)](https://isaac-sim.github.io/IsaacLab/)
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

### Convert urdf to usd
```bash
# urdf files in isaacLab.manipulation/isaacLab/manipulation/assets/urdf 
# usd files in isaacLab.manipulation/isaacLab/manipulation/assets/usd
python3 scripts/tools/convert_urdf.py urdf/your_urdf.urdf usd/your_usd.usd
```
### About isaac nucleus
If you dont want to use the assets in isaac nucleus, please comment the following code and replace it with your own assets path.
```bash
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
# from isaacLab.manipulation.assets.config import your_robot
```
### RL Algorithm

Install [RSL_RL](https://github.com/leggedrobotics/rsl_rl) outside of the isaacLab repository, e.g. `home/code/rsl_rl`.

```bash
mkdir isaacLab.manipulation/isaacLab/manipulation/algorithms
cd isaacLab.manipulation/isaacLab/manipulation/algorithms
git clone https://github.com/leggedrobotics/rsl_rl.git
cd rsl_rl
# IMPORTANT: Use a specific version of rsl_rl that is compatible with this project
git checkout 73fd7c6  # v2.0.1 release
python -m pip install -e .

## refresh index
cd isaacLab.manipulation
python -m pip install -e .
```
You can design your own RL Algorithm by editing "modules" and "algorithms" in RSL-RL

## Usage
```bash
#root of the package
cd isaacLab.manipulation
```
### 1. Convert your urdf to usd
```bash
python3 scripts/tools/convert_urdf.py urdf/your_urdf.urdf usd/your_usd.usd
```
### 2. Set up your robot or objects
```bash
cd isaacLab.manipulation/isaacLab/manipulation/assets/config
touch your_robot.py
# kinova_gipper.py is an example. You can copy the file and change robot_usd = "kinova_robotiq.usd" to your own usd and apply some changes in ArticulationCfg 
```
And add env configs for each tasks
```bash
mkdir isaacLab.manipulation/isaacLab/manipulation/tasks/Robot_arm/your_tasks/config/your_robot_env
# kinova_gripper in reach task is an example
```
### 3. RL settings
You can modify *actions/rewards/observations/events/terminations* in the directory "mdp" to set up your own RL settings and you can manage them uniformly in "env_cfg" outside "mdp". These files modify the functions of the original isaaclab through rewritting and overloading.

### IMPORTANT ISSUES
1.  'RigidBodyView' object has no attribute 'get_accelerations.
<br>Solution: Update Isaac Sim to version 4.1.0.
2.  argument --cpu: conflicting option string: --cpu
<br>Solution: Annotate the following code in *play.py* and *train.py*:
```python
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
```
<br>Then remove "use_gpu = not args_cli.cpu" in the following code:
```python
parse_env_cfg(args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs)
```

### 4. Train a policy.
4.1 RobotArm
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Reach-Kinova-v0 --num_envs 4096 --headless
```
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Reach-Franka-v0 --num_envs 4096 --headless
```
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Reach-UR10-v0 --num_envs 4096 --headless
```
4.2 Dextrous Hand
```bash
python3 scripts/rsl_rl/train.py --task Template-Isaac-Repose-Cube-Allegro-v0 --num_envs 4096 --headless
```


### 5. Play the trained policy.

5.1 RobotArm
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Reach-Kinova-Play-v0 --num_envs 16
```
```bash
# You can also use train.py if you dont need to add some additional configs.
python3 scripts/rsl_rl/play.py --task Template-Isaac-Reach-Franka-Play-v0 --num_envs 16
```
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Reach-UR10-Play-v0 --num_envs 16
```
5.2 Dextrous Hand
```bash
python3 scripts/rsl_rl/play.py --task Template-Isaac-Repose-Cube-Allegro-Play-v0 --num_envs 16
```

## Author
**Author: Qiwei Wu<br />
Email: nathan.wuqw@gmail.com**





