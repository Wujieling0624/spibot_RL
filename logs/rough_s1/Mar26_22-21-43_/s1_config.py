from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class S1RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.3] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.23,   # [rad]
            'FL_thigh_joint': 0.2,     # [rad]
            'FL_calf_joint': 0.2,   # [rad]

            'RL_hip_joint': 0.23,   # [rad]
            'RL_thigh_joint': 0.2,   # [rad]
            'RL_calf_joint': 0.2,    # [rad]

            'FR_hip_joint': 0.23 ,  # [rad]
            'FR_thigh_joint': 0.2,     # [rad]
            'FR_calf_joint': -0.2,  # [rad]

            'RR_hip_joint': -0.23,   # [rad]
            'RR_thigh_joint': -0.2,   # [rad]
            'RR_calf_joint': -0.2,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.15
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 6

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/s1/urdf/sipder_robot_rl.urdf'
        name = "s1"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    # class rewards( LeggedRobotCfg.rewards ):
    #     soft_dof_pos_limit = 1.0
    #     base_height_target = 0.3
    #     class scales( LeggedRobotCfg.rewards.scales ):
    #         termination = -0.0 #当任务终止时（例如机器人摔倒），给予的惩罚
    #         torques = -0.0002
    #         dof_pos_limits = -1.0
    #         base_height = -2.0  # 增加对基座高度的奖励
    #         feet_air_time =  0.0
    #         tracking_lin_vel = 1.0 #跟踪目标线速度的奖励。
    #         tracking_ang_vel = 0.5 #跟踪目标角速度的奖励。
    #         ang_vel_xy = -0.05 #在 X 和 Y 轴方向上角速度的惩罚。
    #         lin_vel_z = -1.0 #在 Z 轴方向上线速度的惩罚。
    #         orientation = -0.5 #机器人姿态的惩罚。
    #         dof_vel = -0. #关节速度的惩罚。
    #         dof_acc = -2.5e-7 #关节加速度的惩罚
    #         collision = -1. #碰撞的惩罚。
    #         feet_stumble = -0.0  #脚部绊倒的惩罚。
    #         action_rate = -0.0 #动作变化率的惩罚。
    #         stand_still = 0.0 #静止的惩罚


    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.4
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0
            orientation = -1.0
            stand_still = 1.0
            collision = -1.0
            base_height = -5.0        # 严格限制高度
            feet_air_time = -1.0       # 降低空中时间奖励

class S1RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        max_iterations = 500
        run_name = ''
        experiment_name = 'rough_s1'

  
