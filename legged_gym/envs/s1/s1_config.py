from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class S1RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.3] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.3,   # [rad]
            'FL_thigh_joint': 0.2,     # [rad]
            'FL_calf_joint': 0.2,   # [rad]

            'RL_hip_joint': 0.3,   # [rad]
            'RL_thigh_joint': 0.2,   # [rad]
            'RL_calf_joint': 0.2,    # [rad]

            'FR_hip_joint': 0.3 ,  # [rad]
            'FR_thigh_joint': 0.2,     # [rad]
            'FR_calf_joint': -0.2,  # [rad]

            'RR_hip_joint': -0.3,   # [rad]
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
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.5
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0
            base_height = -5.0        # 严格限制高度
            torques = -0.0002        # 减小扭矩惩罚
            dof_pos_limits = -10.0    # 严格限制关节位置
            feet_air_time = -5.0       # 降低空中时间奖励
            stand_still = 5.0

class S1RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        max_iterations = 700
        run_name = ''
        experiment_name = 'rough_s1'

  
