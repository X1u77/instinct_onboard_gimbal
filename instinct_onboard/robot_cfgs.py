import math

import numpy as np

"""A general configuration file for the robots, shared between different scripts. """


class UnitreeWirelessButtons:
    R1 = 0b00000001  # 1
    L1 = 0b00000010  # 2
    start = 0b00000100  # 4
    select = 0b00001000  # 8
    R2 = 0b00010000  # 16
    L2 = 0b00100000  # 32
    F1 = 0b01000000  # 64
    F2 = 0b10000000  # 128
    A = 0b100000000  # 256
    B = 0b1000000000  # 512
    X = 0b10000000000  # 1024
    Y = 0b100000000000  # 2048
    up = 0b1000000000000  # 4096
    right = 0b10000000000000  # 8192
    down = 0b100000000000000  # 16384
    left = 0b1000000000000000  # 32768


class G1_29Dof_TorsoBase:
    NUM_JOINTS = 29
    NUM_ACTIONS = 29
    joint_map = [
        15,
        22,  # shoulder pitch
        14,  # waist pitch
        16,
        23,  # shoulder roll
        13,  # waist roll
        17,
        24,  # shoulder yaw
        12,  # waist yaw
        18,
        25,  # elbow
        0,
        6,  # hip pitch
        19,
        26,  # wrist roll
        1,
        7,  # hip roll
        20,
        27,  # wrist pitch
        2,
        8,  # hip yaw
        21,
        28,  # wrist yaw
        3,
        9,  # knee
        4,
        10,  # ankle pitch
        5,
        11,  # ankle roll
    ]
    sim_joint_names = [  # NOTE: order matters. This list is the order in simulation.
        "left_shoulder_pitch_joint",  #
        "right_shoulder_pitch_joint",
        "waist_pitch_joint",
        "left_shoulder_roll_joint",  #
        "right_shoulder_roll_joint",
        "waist_roll_joint",
        "left_shoulder_yaw_joint",  #
        "right_shoulder_yaw_joint",
        "waist_yaw_joint",
        "left_elbow_joint",  #
        "right_elbow_joint",
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_wrist_roll_joint",
        "right_wrist_roll_joint",
        "left_hip_roll_joint",  #
        "right_hip_roll_joint",
        "left_wrist_pitch_joint",
        "right_wrist_pitch_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "left_wrist_yaw_joint",  #
        "right_wrist_yaw_joint",
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",  #
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
    ]
    real_joint_names = [  # NOTE: order matters. This list is the order in real robot.
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]
    joint_signs = np.array(
        [
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
        ],
        dtype=np.float32,
    )
    joint_limits_high = np.array(
        [
            2.6704,
            2.6704,
            0.5200,
            2.2515,
            1.5882,
            0.5200,
            2.6180,
            2.6180,
            2.6180,
            2.0944,
            2.0944,
            2.8798,
            2.8798,
            1.9722,
            1.9722,
            2.9671,
            0.5236,
            1.6144,
            1.6144,
            2.7576,
            2.7576,
            1.6144,
            1.6144,
            2.8798,
            2.8798,
            0.5236,
            0.5236,
            0.2618,
            0.2618,
        ],
        dtype=np.float32,
    )
    joint_limits_low = np.array(
        [
            -3.0892,
            -3.0892,
            -0.5200,
            -1.5882,
            -2.2515,
            -0.5200,
            -2.6180,
            -2.6180,
            -2.6180,
            -1.0472,
            -1.0472,
            -2.5307,
            -2.5307,
            -1.9722,
            -1.9722,
            -0.5236,
            -2.9671,
            -1.6144,
            -1.6144,
            -2.7576,
            -2.7576,
            -1.6144,
            -1.6144,
            -0.0873,
            -0.0873,
            -0.8727,
            -0.8727,
            -0.2618,
            -0.2618,
        ],
        dtype=np.float32,
    )
    torque_limits = np.array(
        [  # from urdf and in simulation order
            25,
            25,
            50,
            25,
            25,
            50,
            25,
            25,
            88,
            25,
            25,
            88,
            88,
            25,
            25,
            88,
            88,
            5,
            5,
            88,
            88,
            5,
            5,
            139,
            139,
            50,
            50,
            50,
            50,
        ],
        dtype=np.float32,
    )
    turn_on_motor_mode = [0x01] * 29
    mode_pr = 0
    mode_machine = 5
    """ please check this value from
        https://support.unitree.com/home/zh/G1_developer/basic_services_interface
        https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
    """
    realsense_depth_link_transform = {
        "translation": (
            0.04764571478 + 0.0039635 - 0.0042 * math.cos(math.radians(48)),
            0.015,
            0.46268178553 - 0.044 + 0.0042 * math.sin(math.radians(48)) + 0.016,
        ),
        "rotation": (
            math.cos(math.radians(0.5) / 2) * math.cos(math.radians(48) / 2),  # w
            math.sin(math.radians(0.5) / 2),  # x
            math.sin(math.radians(48) / 2),  # y
            0.0,  # z
        ),
        "parent_frame": "torso_link",
        "child_frame": "realsense_depth_link",
    }


class G1_31Dof_TorsoBase:
    """G1 with 2 additional head gimbal joints (head_yaw, head_pitch) controlled by UART servos.

<<<<<<< HEAD
    Joint order matches the simulation order documented in unitree_g1.py:
      [0-10]  shoulders / waist / elbows
      [11-28] legs / wrists interleaved in IsaacLab articulation order
      [29]    head_yaw   (UART servo, NOT in Unitree LowState)
      [30]    head_pitch (UART servo, NOT in Unitree LowState)
=======
    Joint order matches g1_31dof.urdf used in simulation training:
      [0-11]  Left leg  (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
      [12-23] Right leg (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
      [24]    waist_yaw
      [25]    waist_roll
      [26]    waist_pitch
      [27-32] Left arm  (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw)
      [33-38] Right arm (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw)
      [39]    head_yaw   (UART servo, NOT in Unitree LowState)
      [40]    head_pitch (UART servo, NOT in Unitree LowState)
>>>>>>> 17f938fb4c26a86fa10d2d00379403cbc700d554

    Coordinate system note for head joints:
      - head_yaw:  sim positive = turning left, real servo positive = turning right
                   -> joint_signs[29] = -1
      - head_pitch: sim positive = looking down, real servo positive = looking down
                   -> joint_signs[30] = 1 (same direction)
    """

    NUM_JOINTS = 31
    NUM_ACTIONS = 31
<<<<<<< HEAD
    head_default_joint_pos = np.array([0.0, 0.8726646259971648], dtype=np.float32)
=======
>>>>>>> 17f938fb4c26a86fa10d2d00379403cbc700d554

    # sim_index -> real Unitree motor index
    # -1 means controlled by UART servo, not Unitree LowState
    joint_map = [
        # Left leg  (6 joints)
        15,  # [0]  left_shoulder_pitch_joint  -> real[15]
        22,  # [1]  right_shoulder_pitch_joint -> real[22]
        14,  # [2]  waist_pitch_joint          -> real[14]
        16,  # [3]  left_shoulder_roll_joint   -> real[16]
        23,  # [4]  right_shoulder_roll_joint  -> real[23]
        13,  # [5]  waist_roll_joint          -> real[13]
        17,  # [6]  left_shoulder_yaw_joint   -> real[17]
        24,  # [7]  right_shoulder_yaw_joint  -> real[24]
        12,  # [8]  waist_yaw_joint           -> real[12]
        18,  # [9]  left_elbow_joint          -> real[18]
        25,  # [10] right_elbow_joint         -> real[25]
        0,   # [11] left_hip_pitch_joint     -> real[0]
        6,   # [12] right_hip_pitch_joint    -> real[6]
        19,  # [13] left_wrist_roll_joint    -> real[19]
        26,  # [14] right_wrist_roll_joint   -> real[26]
        1,   # [15] left_hip_roll_joint      -> real[1]
        7,   # [16] right_hip_roll_joint     -> real[7]
        20,  # [17] left_wrist_pitch_joint   -> real[20]
        27,  # [18] right_wrist_pitch_joint  -> real[27]
        2,   # [19] left_hip_yaw_joint       -> real[2]
        8,   # [20] right_hip_yaw_joint      -> real[8]
        21,  # [21] left_wrist_yaw_joint     -> real[21]
        28,  # [22] right_wrist_yaw_joint    -> real[28]
        3,   # [23] left_knee_joint          -> real[3]
        9,   # [24] right_knee_joint         -> real[9]
        4,   # [25] left_ankle_pitch_joint  -> real[4]
        10,  # [26] right_ankle_pitch_joint -> real[10]
        5,   # [27] left_ankle_roll_joint    -> real[5]
        11,  # [28] right_ankle_roll_joint   -> real[11]
        # Head gimbal (2 joints) - UART servo controlled, NOT in Unitree LowState
        -1,  # [29] head_yaw_joint
        -1,  # [30] head_pitch_joint
    ]

    sim_joint_names = [
        # Left leg (6)
        "left_shoulder_pitch_joint",
        "right_shoulder_pitch_joint",
        "waist_pitch_joint",
        "left_shoulder_roll_joint",
        "right_shoulder_roll_joint",
        "waist_roll_joint",
        "left_shoulder_yaw_joint",
        "right_shoulder_yaw_joint",
        "waist_yaw_joint",
        "left_elbow_joint",
        "right_elbow_joint",
        # Legs (12)
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_wrist_roll_joint",
        "right_wrist_roll_joint",
        "left_hip_roll_joint",
        "right_hip_roll_joint",
        "left_wrist_pitch_joint",
        "right_wrist_pitch_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "left_wrist_yaw_joint",
        "right_wrist_yaw_joint",
        # Knees & ankles (6)
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
        # Head gimbal (2)
        "head_yaw_joint",
        "head_pitch_joint",
    ]

    real_joint_names = [
        # Matches sim_joint_names order for the 29 Unitree motors
        "left_shoulder_pitch_joint",
        "right_shoulder_pitch_joint",
        "waist_pitch_joint",
        "left_shoulder_roll_joint",
        "right_shoulder_roll_joint",
        "waist_roll_joint",
        "left_shoulder_yaw_joint",
        "right_shoulder_yaw_joint",
        "waist_yaw_joint",
        "left_elbow_joint",
        "right_elbow_joint",
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_wrist_roll_joint",
        "right_wrist_roll_joint",
        "left_hip_roll_joint",
        "right_hip_roll_joint",
        "left_wrist_pitch_joint",
        "right_wrist_pitch_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "left_wrist_yaw_joint",
        "right_wrist_yaw_joint",
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
        # Head gimbal joints are NOT in Unitree LowState
        "head_yaw_joint",
        "head_pitch_joint",
    ]

    # head_yaw: sim positive=left, real servo positive=right -> sign = -1
    # head_pitch: sim positive=down, real servo positive=down -> sign = +1
    joint_signs = np.array(
        [
            1,   # left_shoulder_pitch
            1,   # right_shoulder_pitch
            -1,  # waist_pitch
            1,   # left_shoulder_roll
            1,   # right_shoulder_roll
            -1,  # waist_roll
            1,   # left_shoulder_yaw
            1,   # right_shoulder_yaw
            -1,  # waist_yaw
            1,   # left_elbow
            1,   # right_elbow
            1,   # left_hip_pitch
            1,   # right_hip_pitch
            1,   # left_wrist_roll
            1,   # right_wrist_roll
            1,   # left_hip_roll
            1,   # right_hip_roll
            1,   # left_wrist_pitch
            1,   # right_wrist_pitch
            1,   # left_hip_yaw
            1,   # right_hip_yaw
            1,   # left_wrist_yaw
            1,   # right_wrist_yaw
            1,   # left_knee
            1,   # right_knee
            1,   # left_ankle_pitch
            1,   # right_ankle_pitch
            1,   # left_ankle_roll
            1,   # right_ankle_roll
            -1,  # head_yaw   (sim=left+, servo=right+)
            1,   # head_pitch  (sim=down+, servo=down+)
        ],
        dtype=np.float32,
    )

    joint_limits_high = np.array(
        [
            2.6704,  # left_shoulder_pitch
            2.6704,  # right_shoulder_pitch
            0.5200,  # waist_pitch
            2.2515,  # left_shoulder_roll
            1.5882,  # right_shoulder_roll
            0.5200,  # waist_roll
            2.6180,  # left_shoulder_yaw
            2.6180,  # right_shoulder_yaw
            2.6180,  # waist_yaw
            2.0944,  # left_elbow
            2.0944,  # right_elbow
            2.8798,  # left_hip_pitch
            2.8798,  # right_hip_pitch
            1.9722,  # left_wrist_roll
            1.9722,  # right_wrist_roll
            2.9671,  # left_hip_roll
            0.5236,  # right_hip_roll
            1.6144,  # left_wrist_pitch
            1.6144,  # right_wrist_pitch
            2.7576,  # left_hip_yaw
            2.7576,  # right_hip_yaw
            1.6144,  # left_wrist_yaw
            1.6144,  # right_wrist_yaw
            2.8798,  # left_knee
            2.8798,  # right_knee
            0.5236,  # left_ankle_pitch
            0.5236,  # right_ankle_pitch
            0.2618,  # left_ankle_roll
            0.2618,  # right_ankle_roll
            2.2680,  # head_yaw   (from URDF)
            1.7450,  # head_pitch  (from URDF)
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.0892,  # left_shoulder_pitch
            -3.0892,  # right_shoulder_pitch
            -0.5200,  # waist_pitch
            -1.5882,  # left_shoulder_roll
            -2.2515,  # right_shoulder_roll
            -0.5200,  # waist_roll
            -2.6180,  # left_shoulder_yaw
            -2.6180,  # right_shoulder_yaw
            -2.6180,  # waist_yaw
            -1.0472,  # left_elbow
            -1.0472,  # right_elbow
            -2.5307,  # left_hip_pitch
            -2.5307,  # right_hip_pitch
            -1.9722,  # left_wrist_roll
            -1.9722,  # right_wrist_roll
            -0.5236,  # left_hip_roll
            -2.9671,  # right_hip_roll
            -1.6144,  # left_wrist_pitch
            -1.6144,  # right_wrist_pitch
            -2.7576,  # left_hip_yaw
            -2.7576,  # right_hip_yaw
            -1.6144,  # left_wrist_yaw
            -1.6144,  # right_wrist_yaw
            -0.0873,  # left_knee
            -0.0873,  # right_knee
            -0.8727,  # left_ankle_pitch
            -0.8727,  # right_ankle_pitch
            -0.2618,  # left_ankle_roll
            -0.2618,  # right_ankle_roll
            -2.2680,  # head_yaw   (from URDF)
            -1.0000,  # head_pitch  (from URDF)
        ],
        dtype=np.float32,
    )

    torque_limits = np.array(
        [
            25,  # left_shoulder_pitch
            25,  # right_shoulder_pitch
            50,  # waist_pitch
            25,  # left_shoulder_roll
            25,  # right_shoulder_roll
            50,  # waist_roll
            25,  # left_shoulder_yaw
            25,  # right_shoulder_yaw
            88,  # waist_yaw
            25,  # left_elbow
            25,  # right_elbow
            88,  # left_hip_pitch
            88,  # right_hip_pitch
            25,  # left_wrist_roll
            25,  # right_wrist_roll
            88,  # left_hip_roll
            88,  # right_hip_roll
            5,   # left_wrist_pitch
            5,   # right_wrist_pitch
            88,  # left_hip_yaw
            88,  # right_hip_yaw
            5,   # left_wrist_yaw
            5,   # right_wrist_yaw
            139, # left_knee
            139, # right_knee
            50,  # left_ankle_pitch
            50,  # right_ankle_pitch
            50,  # left_ankle_roll
            50,  # right_ankle_roll
            9,   # head_yaw   (servo effort limit from URDF)
            9,   # head_pitch  (servo effort limit from URDF)
        ],
        dtype=np.float32,
    )

    # Motor mode for Unitree motors (index 0-28). Head joints (-1) are not sent to Unitree SDK.
    turn_on_motor_mode = [0x01] * 29 + [0x00] * 2
    mode_pr = 0
    mode_machine = 5

    realsense_depth_link_transform = {
        "translation": (
            0.04764571478 + 0.0039635 - 0.0042 * math.cos(math.radians(48)),
            0.015,
            0.46268178553 - 0.044 + 0.0042 * math.sin(math.radians(48)) + 0.016,
        ),
        "rotation": (
            math.cos(math.radians(0.5) / 2) * math.cos(math.radians(48) / 2),  # w
            math.sin(math.radians(0.5) / 2),  # x
            math.sin(math.radians(48) / 2),  # y
            0.0,  # z
        ),
        "parent_frame": "torso_link",
        "child_frame": "realsense_depth_link",
    }
