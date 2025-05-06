# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

#import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
import isaacLab.manipulation.tasks.Robot_arm.reach.mdp as mdp
from isaacLab.manipulation.tasks.Robot_arm.reach.reach_env_cfg import ReachEnvCfg
#from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaacLab.manipulation.assets.config.kinova_gripper import KINOVA_ROBOTIQ # isort: skip


##
# Environment configuration
##


@configclass
class KinovaReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to ur10
        self.scene.robot = KINOVA_ROBOTIQ.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.replicate_physics=False
        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["end_effector_link"]
        # override actions
        self.actions.arm_action = mdp.RelativeJointPositionActionCfg(
            asset_name="robot", joint_names=["joint_[1-7]"], scale=0.04
        )

        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "end_effector_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)


@configclass
class KinovaReachEnvCfg_PLAY(KinovaReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.scene.replicate_physics=False
        # disable randomization for play
        self.observations.policy.enable_corruption = False
