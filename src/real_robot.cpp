#include "real_robot.h"

#include <iostream>

#include "nlohmann/json/json.hpp"
#include "pconfig.hpp"
#include "putil.h"

RealRobot::RealRobot() {
  joint_interface_ = std::make_unique<JointInterface>();
  hands_position = std::vector<int>(kHandsLinearActuatorDof, 0);
}

RealRobot::~RealRobot() { imu_.close(); }

AdamStatusCode RealRobot::init() {
  auto res = imu_.initialize();
  if (!res) {
    std::cout << "initIMU failed" << std::endl;
    return AdamStatusCode::AdamStatusFailure;
  }

  std::vector<std::string> ankle_names = {"anklePitch_Left", "ankleRoll_Left", "anklePitch_Right", "ankleRoll_Right"};
  ankle_ids_ = PConfig::getInst().jointIdFromNames(ankle_names);
  std::vector<int> ankle_nopose_ids(4);
  for (size_t i = 0; i < ankle_ids_.size(); i++) {
    if (ankle_ids_[i] == -1) {
      std::cout << "real robot ERROR: joint " << ankle_names[i] << " not found" << std::endl;
      exit(1);
    }
    ankle_nopose_ids[i] = ankle_ids_[i];
    ankle_ids_[i] += kBaseNum;
  }

#if defined(ADAM_STANDARD) || defined(ADAM_INSPIRE)
  std::vector<std::string> wrist_names = {"wristPitch_Left", "wristRoll_Left", "wristPitch_Right", "wristRoll_Right"};
  wrist_ids_ = PConfig::getInst().jointIdFromNames(wrist_names);
  for (size_t i = 0; i < wrist_ids_.size(); i++) {
    if (wrist_ids_[i] == -1) {
      std::cout << "real robot ERROR: joint " << wrist_names[i] << " not found" << std::endl;
      exit(1);
    }
    wrist_ids_[i] += kBaseNum;
  }
#endif

  Eigen::VectorXd absolute_pos_zero = PConfig::getInst().absolutePosZero();
  Eigen::VectorXd absolute_pos_init = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd motor_enc_cur_pos = Eigen::VectorXd::Zero(kRobotDof);

  readAbsEncoder(absolute_pos_init, motor_enc_cur_pos);

  // abs set
  Eigen::VectorXd absolute_pos = Eigen::VectorXd::Zero(kRobotDof);
  Eigen::VectorXd absolute_pos_gear_ratio = PConfig::getInst().absolutePosGearRatio();
  Eigen::VectorXd abs_pos_dir = PConfig::getInst().absolutePosDir();
  absolute_pos = absolute_pos_init - absolute_pos_zero;

  for (int i = 0; i < kRobotDof; i++) {
    if (absolute_pos(i) < -M_PI) {
      absolute_pos(i) = absolute_pos(i) + 2.0 * M_PI;
    } else if (absolute_pos(i) > M_PI) {
      absolute_pos(i) = absolute_pos(i) - 2.0 * M_PI;
    }
    absolute_pos(i) = absolute_pos(i) / (abs_pos_dir[i] * absolute_pos_gear_ratio(i));
  }
  std::cout << "absolute_pos_gear_ratio: " << absolute_pos_gear_ratio.transpose() << std::endl;
  std::cout << "abs_pos_dir: " << abs_pos_dir.transpose() << std::endl;
  std::cout << "absolute_pos: " << absolute_pos.transpose() << std::endl;

  PndAbsS2P(absolute_pos, ankle_nopose_ids);

  joint_Kp_s = joint_Kp_;
  joint_Kd_s = joint_Kd_;

  for (int i = 0; i < kRobotDof; i++) {
    joint_Kp_(i) = joint_Kp_(i) / joint_Kd_(i);
    joint_Kd_(i) = joint_Kd_(i) / PConfig::getInst().kdScale()(i);
  }

  auto roll_idxs = PConfig::getInst().jointIdFromNames({"ankleRoll_Left", "ankleRoll_Right"});
  auto pitch_idxs = PConfig::getInst().jointIdFromNames({"anklePitch_Left", "anklePitch_Right"});
  for (size_t i = 0; i < roll_idxs.size(); ++i) {
    joint_Kp_(roll_idxs[i]) = joint_Kp_(pitch_idxs[i]);
    joint_Kd_(roll_idxs[i]) = joint_Kd_(pitch_idxs[i]);
  }

  auto joint_names = PConfig::getInst().jointNames();
  auto joint_gear_ratio = PConfig::getInst().jointGearRatio();
  auto joint_dir = PConfig::getInst().jointDir();
  auto motor_enc_zero = PConfig::getInst().motorRotorAbsPos();
  auto abs_pos_gear_ratio = PConfig::getInst().absolutePosGearRatio();
  auto abs_pos_zero = PConfig::getInst().absolutePosZero();
  Eigen::VectorXd min_end_detect_pos = Eigen::VectorXd::Zero(kRobotDof);
  // clang-format off
  min_end_detect_pos << -2.09, -0.78, -0.78, -0.09, -1, -0.3491,
                        -2.09, -1.57, -0.78, -0.09, -1, -0.3491,
                        -0.52, -0.78, -0.78;
  // clang-format on

  auto check_zero_idx = PConfig::getInst().jointIdFromNames(
      {"hipRoll_Left", "hipYaw_Left", "hipPitch_Right", "hipRoll_Right", "hipYaw_Right", "waistYaw"});
  // auto check_zero_idx = PConfig::getInst().jointIdFromNames(
  //     {"hipPitch_Left", "hipRoll_Left", "hipYaw_Left", "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
  //     "waistYaw"});
  for (const auto i : check_zero_idx) {
    double end_pos = 0.0;
    std::cout << joint_names[i] << ":" << joint_gear_ratio(i) << " " << abs_pos_gear_ratio(i) << " "
              << radToDeg(min_end_detect_pos(i)) << " " << radToDeg(motor_enc_zero(i)) << " "
              << radToDeg(abs_pos_zero(i)) << " " << joint_dir(i) << " " << abs_pos_dir(i) << " "
              << radToDeg(motor_enc_cur_pos(i)) << " " << radToDeg(absolute_pos_init(i)) << std::endl;

    PndAbsCheck(joint_gear_ratio(i), abs_pos_gear_ratio(i), radToDeg(min_end_detect_pos(i)), 1.0, 0, 0,
                radToDeg(motor_enc_zero(i)), radToDeg(abs_pos_zero(i)), joint_dir(i), abs_pos_dir(i),
                radToDeg(motor_enc_cur_pos(i)), radToDeg(absolute_pos_init(i)), end_pos);
    std::cout << end_pos << " " << std::endl;
    if (!std::isnan(end_pos) && end_pos < 180 / abs_pos_gear_ratio(i) && end_pos > -180 / abs_pos_gear_ratio(i)) {
      std::cout << joint_names[i] << " check zero success" << std::endl;
    } else {
      std::cout << joint_names[i] << " check zero failed" << std::endl;
      exit(1);
    }
  }

  // std::cout << "joint_Kp_: " << joint_Kp_.transpose() << std::endl;
  // std::cout << "joint_Kd_: " << joint_Kd_.transpose() << std::endl;

  // joint_Kp_.setZero();
  // joint_Kd_.setZero();
  //
  // joint_Kp_ *= 0.1;
  // joint_Kd_ *= 0.1;

  joint_interface_->init(absolute_pos, joint_Kp_, joint_Kd_);

  max_tau_ = (PConfig::getInst().jointMaxCurrent().array() * PConfig::getInst().jointGearRatio().array() *
              PConfig::getInst().curTorScale().array() / PConfig::getInst().kdScale().array())
                 .matrix();
  kp_mul_kd_ = joint_Kp_.array() * joint_Kd_.array();

  if (PConfig::getInst().handType() == HandType::PND_HAND) {
    hands_ctrl_ = getPndHandInterface({1800, 1800, 1800, 1800, 1600, 0});
    if (hands_ctrl_ == nullptr) {
      std::cout << "ERROR: PndHandInterface init failed" << std::endl;
      return AdamStatusCode::AdamStatusFailure;
    }
  }
  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode RealRobot::getState(double t, RobotData& robot_data) {
  robot_data.imu_data_ = imu_.getImuData();
  joint_interface_->getState(joint_pos_, joint_vel_, joint_tau_);
  robot_data.error_state_ = joint_interface_->joint_error_;
  robot_data.q_a_.tail(kRobotDof) = joint_pos_;
  robot_data.q_dot_a_.tail(kRobotDof) = joint_vel_;
  robot_data.tau_a_.tail(kRobotDof) = joint_tau_;
  // q_a_, q_dot_a_, tau_a_ trans to serial
  PndP2S(robot_data, ankle_ids_);
#if defined(ADAM_STANDARD) || defined(ADAM_INSPIRE)
  PndWristP2S(robot_data, wrist_ids_);
#endif

  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode RealRobot::setCommand(RobotData& robot_data) {
  // S2P
  PndS2P(robot_data, PConfig::getInst().kdScale(), joint_Kp_, joint_Kd_, joint_Kp_s, joint_Kd_s, ankle_ids_);
#if defined(ADAM_STANDARD) || defined(ADAM_INSPIRE)
  PndWristS2P(robot_data, wrist_ids_);
#endif

  if (robot_data.clip_qd_) {
    max_qd_ = ((max_tau_ - joint_Kd_ * (-robot_data.q_dot_a_.tail(kRobotDof))).array() / kp_mul_kd_).matrix() +
              robot_data.q_a_.tail(kRobotDof);

    min_qd_ = ((-max_tau_ - joint_Kd_ * (-robot_data.q_dot_a_.tail(kRobotDof))).array() / kp_mul_kd_).matrix() +
              robot_data.q_a_.tail(kRobotDof);

    for (int i = 0; i < kRobotDof; i++) {
      if (robot_data.q_d_(i + kBaseNum) > max_qd_(i)) {
        std::cout << "joint " << i + 1 << " qd is clipped to +max" << std::endl;
        robot_data.q_d_(i + kBaseNum) = max_qd_(i);
      } else if (robot_data.q_d_(i + kBaseNum) < min_qd_(i)) {
        std::cout << "joint " << i + 1 << " qd is clipped to -max" << std::endl;
        robot_data.q_d_(i + kBaseNum) = min_qd_(i);
      }
    }
  }

  // send command
  joint_interface_->setCommand(robot_data.q_d_.tail(kRobotDof), robot_data.q_dot_d_.tail(kRobotDof),
                               robot_data.tau_d_.tail(kRobotDof));

  // hands control
  if (PConfig::getInst().handType() == HandType::PND_HAND) {
    static int freq_ = 0;
    if (freq_ % 8 == 0) {
      for (int i = 0; i < robot_data.hands_la_q_d_.size(); i++) {
        hands_position[i] = static_cast<int>(robot_data.hands_la_q_d_(i));
      }
      for (int i = 0; i < robot_data.hands_la_q_d_.size(); i++) {
        hands_position[i] = (1000 - hands_position[i]) * 2;
        if (hands_position[i] >= 1800) {
          hands_position[i] = 1800;
        }
      }
      hands_ctrl_->setPosition(hands_position);
      freq_ = 0;
    }
    freq_++;
  }

  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode RealRobot::disableAllJoints() {
  joint_interface_->disable();
  return AdamStatusCode::AdamStatusSuccess;
}

AdamStatusCode RealRobot::readAbsEncoder(Eigen::VectorXd& init_pos, Eigen::VectorXd& motor_enc_init_pos) {
  std::ifstream abs_file("python_scripts/source/abs.json");
  nlohmann::json data;
  if (!abs_file.is_open()) {
    std::cout << "python_scripts/source/abs.json not found" << std::endl;
    exit(1);
  }
  try {
    data = nlohmann::json::parse(abs_file);
  } catch (std::exception& e) {
    std::cout << "abs.json parse error" << std::endl;
    std::cout << e.what() << std::endl;
  }
  abs_file.close();

  std::vector<std::string> keys = PConfig::getInst().ipList();
  incrementLastField(keys);

  for (auto& item : data.items()) {
    if (!item.value().contains("radian") || !item.value().contains("motor_rotor_abs_pos")) {
      std::cout << item.key() << " not used" << std::endl;
      continue;
    }
    auto it = std::find(keys.begin(), keys.end(), item.key());
    if (it != keys.end()) {
      int idx = std::distance(keys.begin(), it);
      init_pos[idx] = item.value()["radian"].get<double>();
      motor_enc_init_pos[idx] = item.value()["motor_rotor_abs_pos"].get<double>();
    } else {
      std::cout << item.key() << " not used" << std::endl;
    }
  }
  return AdamStatusCode::AdamStatusSuccess;
}
