#ifndef PND_ALGORITHM_H_
#define PND_ALGORITHM_H_

#include <Eigen/Dense>
#include <vector>

#define DATALOG  // for data log

constexpr int kBaseNum = 6;  // floating base
#ifdef ADAM_LITE
constexpr int kRobotDof = 25;  // robot dof number
#elif defined(ADAM_STANDARD)
constexpr int kRobotDof = 31;
#elif defined(ADAM_INSPIRE)
constexpr int kRobotDof = 29;
#elif defined(ADAM_SP)
constexpr int kRobotDof = 29;
#elif defined(ADAM_PRO)
constexpr int kRobotDof = 31;
#endif

#if defined(ADAM_PRO) || defined(ADAM_SP)
constexpr int kHandsDof = 24;  // hand dof number
constexpr int kHandsLinearActuatorDof = 12;
#else
constexpr int kHandsDof = 0;
constexpr int kHandsLinearActuatorDof = 0;
#endif

constexpr int kObsDof = 23;                           // observation dof number
constexpr int kRobotDataSize = kBaseNum + kRobotDof;  // robot data size

typedef enum PndAlgorithmStatusCode {
  PndAlgorithmSuccess = 0,
  PndAlgorithmError = 1,
} PndAlgorithmStatusCode;

struct RobotData {
  // vector description:
  // [0]:floating base x
  // [1]:floating base y
  // [2]:floating base z
  // [3]:floating base roll
  // [4]:floating base pitch
  // [5]:floating base yaw
  // [6:]:ref to pconfig.hpp -> PConfig()

  // actual position
  Eigen::VectorXd q_a_ = Eigen::VectorXd::Zero(kRobotDataSize);
  // actual velocity
  Eigen::VectorXd q_dot_a_ = Eigen::VectorXd::Zero(kRobotDataSize);
  // actual torque
  Eigen::VectorXd tau_a_ = Eigen::VectorXd::Zero(kRobotDataSize);

  // desired position
  Eigen::VectorXd q_d_ = Eigen::VectorXd::Zero(kRobotDataSize);
  // desired velocity
  Eigen::VectorXd q_dot_d_ = Eigen::VectorXd::Zero(kRobotDataSize);
  // desired torque
  Eigen::VectorXd tau_d_ = Eigen::VectorXd::Zero(kRobotDataSize);

  // vector description: yaw pitch roll gyro_x gyro_y gyro_z acc_x acc_y acc_z
  Eigen::VectorXd imu_data_ = Eigen::VectorXd::Zero(9);

  Eigen::VectorXd hands_q_d_ = Eigen::VectorXd::Zero(kHandsDof);      // hands desired position for mujoco
  Eigen::VectorXd hands_q_a_ = Eigen::VectorXd::Zero(kHandsDof);      // hands actual position for mujoco
  Eigen::VectorXd hands_q_dot_a_ = Eigen::VectorXd::Zero(kHandsDof);  // hands actual velocity for mujoco
  Eigen::VectorXd hands_la_q_d_ =
      Eigen::VectorXd::Zero(kHandsLinearActuatorDof);  // hands linear actuator desired position for real robot

  bool pos_mode_ = true;

  bool error_state_ = false;

  bool clip_qd_ = false;
#ifdef DATALOG
  Eigen::VectorXd dataL = Eigen::VectorXd::Zero(350);
#endif
};

/**
 * @brief Initialize the algorithm.
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 */
PndAlgorithmStatusCode PndStateEstimateInit();

/**
 * @brief State estimation algorithm.
 * Update floating base position: q_a_[0-2] (x,y updated to 0.0)
 * Update floating base orientation: q_a_[3-5]
 * Update floating base linear velocity: q_dot_a_[0-2]
 * Update floating base angular velocity: q_dot_a_[3-5]
 * @param t Time stamp
 * @param robot_data Global robot data structure
 * @param leg_pos legged joint position
 * @param leg_vel legged joint velocity
 * @param leg_tau legged joint torque
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 */
PndAlgorithmStatusCode PndStateEstimate(double t, RobotData& robot_data, const Eigen::VectorXd& leg_pos,
                                        const Eigen::VectorXd& leg_vel, const Eigen::VectorXd& leg_tau);

/**
 * @brief Ankle absolute Parallel to Serial
 * @param absolute_pos Absolute position
 * @param ankle_ids Ankle joint ids
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 */
PndAlgorithmStatusCode PndAbsS2P(Eigen::VectorXd& absolute_pos, const std::vector<int>& ankle_ids);

/**
 * @brief Ankle Parallel to Serial
 * @param robot_data Global robot data structure
 * @param ankle_ids Ankle joint ids
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 */
PndAlgorithmStatusCode PndP2S(RobotData& robot_data, const std::vector<int>& ankle_ids);

/**
 * @brief Ankle Serial to Parallel
 * @param robot_data Global robot data structure
 * @param kd_scale
 * @param joint_Kp
 * @param joint_Kd
 * @param joint_Kp_s
 * @param joint_Kd_s
 * @param ankle_ids
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 */
PndAlgorithmStatusCode PndS2P(RobotData& robot_data, const Eigen::VectorXd& kd_scale, const Eigen::VectorXd& joint_Kp,
                              const Eigen::VectorXd& joint_Kd, const Eigen::VectorXd& joint_Kp_s,
                              const Eigen::VectorXd& joint_Kd_s, const std::vector<int>& ankle_ids);

PndAlgorithmStatusCode PndWristP2S(RobotData& robot_data, const std::vector<int>& wrist_ids);
PndAlgorithmStatusCode PndWristS2P(RobotData& robot_data, const std::vector<int>& wrist_ids);

PndAlgorithmStatusCode PndAbsCheck(double motor_rotor_enc_gear_ratio, double end_enc_gear_ratio,
                                   double min_end_detect_pos, double detect_accuracy, double motor_rotor_enc_noise,
                                   double end_enc_noise, double motor_rotor_enc_offset, double end_enc_offset,
                                   int motor_rotor_enc_dir, int end_enc_dir, double motor_rotor_enc_cur_pos,
                                   double end_enc_cur_pos, double& end_pos);

/**
 * @brief Release the resources.
 * @return PndAlgorithmSuccess on success，PndAlgorithmError on failed.
 * @note This function should be called before the program exits.
 */
PndAlgorithmStatusCode PndAlgorithmRelease();

#endif  // PND_ALGORITHM_H_