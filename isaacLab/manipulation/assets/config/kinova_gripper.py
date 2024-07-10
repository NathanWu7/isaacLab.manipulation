# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Kinova Robotics arms.

The following configuration parameters are available:

* :obj:`KINOVA_JACO2_N7S300_CFG`: The Kinova JACO2 (7-Dof) arm with a 3-finger gripper.
* :obj:`KINOVA_JACO2_N6S300_CFG`: The Kinova JACO2 (6-Dof) arm with a 3-finger gripper.
* :obj:`KINOVA_GEN3_N7_CFG`: The Kinova Gen3 (7-Dof) arm with no gripper.

Reference: https://github.com/Kinovarobotics/kinova-ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
usd_dir_path = os.path.join(BASE_DIR, "../usd/")

robot_usd = "kinova_robotiq.usd"

##
# Configuration
##

KINOVA_ROBOTIQ = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_dir_path + robot_usd,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
            fix_root_link = True
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.785,
            "joint_3": 3.14159,
            "joint_4": 1.4,
            "joint_5": 0.0,
            "joint_6": 1.57,
            "joint_7": 1.57,
            "finger_joint": 0.0,
            "left_inner_knuckle_joint": 0.0,
            "left_inner_finger_joint": 0.0,
            "right_outer_knuckle_joint": 0.0,
            "right_inner_knuckle_joint": 0.0,
            "right_inner_finger_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            velocity_limit=1.2,
            effort_limit={
                "joint_[1-4]": 39.0,
                "joint_[5-7]": 9.0,
            },
            stiffness={
                "joint_[1-4]": 300.0,  #100
                "joint_[5-7]": 200.0,   # 15
            },
            damping={
                "joint_[1-4]": 10.0,    # 1.0
                "joint_[5-7]": 5.0,    #0.5
            },
        ),
        "hand": ImplicitActuatorCfg(
            joint_names_expr=["right_outer_knuckle_joint","finger_joint"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=10.0,
            damping=10.0,
        ),
        "hand_mimic": ImplicitActuatorCfg(
            joint_names_expr=["left_inner_knuckle_joint","left_inner_finger_joint",
            "right_inner_knuckle_joint","right_inner_finger_joint"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=0.0,
            damping=5000.0,
        ),
    },
)

"""Configuration of robot with stiffer PD control."""
KINOVA_ROBOTIQ_HPD = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_dir_path + robot_usd,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
            fix_root_link = True
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.785,
            "joint_3": 3.14159,
            "joint_4": 1.4,
            "joint_5": 0.0,
            "joint_6": 0.0,
            "joint_7": -1.57,
            "finger_joint": 0.0,
            "left_inner_knuckle_joint": 0.0,
            "left_inner_finger_joint": 0.0,
            "right_outer_knuckle_joint": 0.0,
            "right_inner_knuckle_joint": 0.0,
            "right_inner_finger_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            velocity_limit=2.0,
            effort_limit={
                "joint_[1-4]": 39.0,
                "joint_[5-7]": 9.0,
            },
            stiffness={
                "joint_[1-4]": 600.0,  #100
                "joint_[5-7]": 450.0,   # 15
            },
            damping={
                "joint_[1-4]": 30.0,    # 1.0
                "joint_[5-7]": 15.0,    #0.5
            },
        ),
        "hand": ImplicitActuatorCfg(
            joint_names_expr=["left_inner_knuckle_joint","left_inner_finger_joint",
            "right_inner_knuckle_joint","right_outer_knuckle_joint","right_inner_finger_joint","finger_joint"],
            effort_limit=150.0,
            velocity_limit=2.24,
            stiffness=5.0,
            damping=5.0,
        ),
    },
)
