#include "fsm/fsm_mlp_baseline_demo.h"

#include <torch/script.h>

#include "pconfig.hpp"
#include "pdata_handler.hpp"
#include "putil.h"

std::map<GaitEnum, double> gait_map;

StateMLP::StateMLP(RobotData *robot_data) : FSMState(robot_data) {
  current_state_name_ = FSMStateName::MLP;

  gait_map[GaitEnum::Stand] = 0.0;
  gait_map[GaitEnum::Walk] = 1.0;
  gait_map[GaitEnum::Run] = 2.0;

  input_data_mlp = Eigen::VectorXd::Zero(PConfig::getInst().obsNum());
  command_scales[0] = obs_scales_lin_vel;
  command_scales[1] = obs_scales_lin_vel;
  command_scales[2] = obs_scales_ang_vel;
  omega_filter = new LowPassFilter(30, 0.707, 0.0025, 3);
  input_array = new float[PConfig::getInst().obsNum()];
}

StateMLP::~StateMLP() {
  delete[] input_array;
  delete omega_filter;
}

void StateMLP::onEnter() {
  std::vector<std::string> obs_joint_names = {
      "hipPitch_Left",       "hipRoll_Left",       "hipYaw_Left",       "kneePitch_Left",  "anklePitch_Left",
      "ankleRoll_Left",  //
      "hipPitch_Right",      "hipRoll_Right",      "hipYaw_Right",      "kneePitch_Right", "anklePitch_Right",
      "ankleRoll_Right",                                                               //
      "waistRoll",           "waistPitch",         "waistYaw",                         //
      "shoulderPitch_Left",  "shoulderRoll_Left",  "shoulderYaw_Left",  "elbow_Left",  //
      "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right"};
  obs_joint_ids_ = PConfig::getInst().jointIdFromNames(obs_joint_names);
  for (size_t i = 0; i < obs_joint_ids_.size(); i++) {
    if (obs_joint_ids_[i] == -1) {
      std::cout << "mlp ERROR: joint " << obs_joint_names[i] << " not found" << std::endl;
    }
  }
  def_dof_pos_ = PConfig::getInst().defDofPosFromIds(obs_joint_ids_);
  zero_pos_ = PConfig::getInst().zeroPosFromIds(obs_joint_ids_);

  std::vector<std::string> wrist_yaw_joint_names = {"wristYaw_Left", "wristYaw_Right"};
  wrist_yaw_ids_ = PConfig::getInst().jointIdFromNames(wrist_yaw_joint_names);

  timer_ = 0.;
  timer_plan = 0.;
  timer_gait = 0.;
  trans_time = 1.0;
  left_phase = left_theta_offset;
  right_phase = right_theta_offset;
  action_last.setZero();
  joint_pos_last_gait = zero_pos_;
  joint_vel_last_gait.setZero();
  last_action_d.setZero();
  last_action_dot_d.setZero();
  input_data_mlp.setZero();
  x_vel_command_offset = 0.0;
  y_vel_command_offset = 0.0;
  robot_data_->clip_qd_ = false;
}

void StateMLP::run() {
  // std::cout << "cnt: " << timer_ / kDt << std::endl;
  // command
  // gait_d = JsHum::getInst().getCommandGait();

  if (gait_a != GaitEnum::Stand) {
    x_vel_command_offset += JsHum::getInst().getWalkXDirectionSpeedOffset();
    y_vel_command_offset += JsHum::getInst().getWalkYDirectionSpeedOffset();

    joystick_command(0) = JsHum::getInst().getWalkXDirectionSpeed();
    joystick_command(1) = JsHum::getInst().getWalkYDirectionSpeed();
    joystick_command(2) = JsHum::getInst().getWalkYawDirectionSpeed();

    joystick_command(0) += x_vel_command_offset;
    joystick_command(1) += y_vel_command_offset;
  }

  if ((fabs(joystick_command(0)) > 0.1)) {
    if ((fabs(joystick_command(0) - command(0)) > 0.0009)) {
      command(0) += 0.0009 * (joystick_command(0) - command(0)) / fabs(joystick_command(0) - command(0));
    } else {
      command(0) = joystick_command(0);
    }
  } else {
    if ((fabs(joystick_command(0) - command(0)) > 0.0015)) {
      command(0) += 0.0015 * (joystick_command(0) - command(0)) / fabs(joystick_command(0) - command(0));
    } else {
      command(0) = joystick_command(0);
    }
  }
  command(1) = joystick_command(1);
  if ((fabs(joystick_command(2) - command(2)) > 0.001) && (fabs(joystick_command(2)) > 0.1)) {
    command(2) += 0.001 * (joystick_command(2) - command(2)) / fabs(joystick_command(2) - command(2));
  } else {
    command(2) = joystick_command(2);
  }

  if (gait_a == GaitEnum::Stand) {
    command.setZero();
    joystick_command.setZero();
  }

  // MLP obs
  Eigen::Matrix3d Rb_w = Eigen::Matrix3d::Identity();
  eulerXYZToMatrix(Rb_w, robot_data_->q_a_.segment(3, 3));
  input_data_mlp.segment(0, 3) = Rb_w.transpose() * robot_data_->q_dot_a_.segment(0, 3) * obs_scales_lin_vel;
  Eigen::Vector3d rpy = robot_data_->q_a_.segment(3, 3);
  Eigen::Matrix3d R_xyz_omega = Eigen::Matrix3d::Identity();
  R_xyz_omega.row(1) = rotX(rpy(0)).row(1);
  R_xyz_omega.row(2) = (rotX(rpy(0)) * rotY(rpy(1))).row(2);
  input_data_mlp.segment(3, 3) =
      Rb_w.transpose() * R_xyz_omega * robot_data_->q_dot_a_.segment(3, 3) * obs_scales_ang_vel;
  input_data_mlp.segment(3, 3) = omega_filter->mFilter(input_data_mlp.segment(3, 3));
  input_data_mlp.segment(6, 3) = -Rb_w.transpose().col(2);
  input_data_mlp(9) = command[0] * command_scales[0];
  input_data_mlp(10) = command[1] * command_scales[1];
  input_data_mlp(11) = command[2] * command_scales[2];
  getInfoFormJointIds(robot_data_->q_a_, obs_joint_ids_, cur_joint_pos_);
  input_data_mlp.segment(12, kObsDof) = (cur_joint_pos_ - def_dof_pos_) * obs_scales_dof_pos;
  getInfoFormJointIds(robot_data_->q_dot_a_, obs_joint_ids_, cur_joint_vel_);
  input_data_mlp.segment(35, kObsDof) = cur_joint_vel_ * obs_scales_dof_vel;
  clip(action_last, -100.0, 100.0);
  input_data_mlp.segment(58, kObsDof) = action_last;
  input_data_mlp.segment(81, 6) =
      gaitPhase(timer_gait, gait_cycle, left_theta_offset, right_theta_offset, left_phase_ratio, right_phase_ratio);
  input_data_mlp(87) = clip((robot_data_->q_a_(2) - 0.8), -1.0, 1.0) * height_scales;
  input_data_mlp(88) =
      (2.0 * kDt / gait_cycle) * input_data_mlp(0) + (1.00 - 2.0 * kDt / gait_cycle) * input_data_mlp(88);
  input_data_mlp(89) =
      (1.0 * kDt / gait_cycle) * input_data_mlp(1) + (1.00 - 1.0 * kDt / gait_cycle) * input_data_mlp(89);
  input_data_mlp(90) =
      (1.0 * kDt / gait_cycle) * input_data_mlp(5) + (1.0 - 1.0 * kDt / gait_cycle) * input_data_mlp(90);

  // link input_data_mlp and inputs
  std::vector<torch::jit::IValue> inputs;
  auto input_data = torch::zeros(PConfig::getInst().obsNum()).toType(torch::kFloat);
  for (int i = 0; i < PConfig::getInst().obsNum(); i++) {
    input_array[i] = input_data_mlp(i);
  }
  input_data = torch::from_blob(input_array, PConfig::getInst().obsNum());
  input_data.to(torch::kCPU);
  inputs.emplace_back(input_data);

  if (gait_a == GaitEnum::Stand && timer_gait >= 1.5) {
    if ((abs(command[0]) <= 0.05)) {
      if (action_last[4] <= 3.0 && action_last[4] >= 1.0 && action_last[10] <= 3.0 && action_last[10] >= 1.0) {
        JsHum::getInst().changeCommandGait(GaitEnum::Stand);
      } else {
        JsHum::getInst().changeCommandGait(GaitEnum::Walk);
      }
    } else {
      JsHum::getInst().changeCommandGait(GaitEnum::Walk);
    }
  }

  gait_d = JsHum::getInst().getCommandGait();

  left_phase = (timer_gait / gait_cycle + left_theta_offset) - floor(timer_gait / gait_cycle + left_theta_offset);
  right_phase = (timer_gait / gait_cycle + right_theta_offset) - floor(timer_gait / gait_cycle + right_theta_offset);
  if (gait_d != gait_a) {
    if (gait_d == GaitEnum::Stand && abs(left_phase - left_phase_ratio) < 0.01) {
      // from walk/run to stand
      timer_gait = 0.0;
      trans_time = 0.1;
      gait_cycle = 1.0;
      left_phase_ratio = 0.0;
      right_phase_ratio = 0.0;
      left_theta_offset = 0.35;
      right_theta_offset = 0.35;
      gait_a = GaitEnum::Stand;
      std::cout << "change to stand!" << std::endl;
    } else if (gait_a == GaitEnum::Stand) {
      // from stand to walk/run
      timer_gait = 0.0;
      if (gait_d == GaitEnum::Walk) {
        trans_time = 0.4;
        gait_cycle = 1.0;          // 0.9;//
        left_phase_ratio = 0.36;   // 0.4;//
        right_phase_ratio = 0.36;  // 0.4;//
        left_theta_offset = left_phase_ratio;
        right_theta_offset = left_theta_offset + 0.5;
        right_theta_offset = right_theta_offset - floor(right_theta_offset);
        gait_a = GaitEnum::Walk;
        std::cout << "change to walk!" << std::endl;
      } else if (gait_d == GaitEnum::Run) {
        trans_time = 1.0;
        gait_cycle = 0.64;
        left_phase_ratio = 0.6;
        right_phase_ratio = 0.6;
        left_theta_offset = left_phase_ratio;
        right_theta_offset = left_theta_offset + 0.5;
        right_theta_offset = right_theta_offset - floor(right_theta_offset);
        gait_a = GaitEnum::Run;
        std::cout << "change to run!" << std::endl;
      }
    } else if (gait_d == GaitEnum::Walk && abs(left_phase - (left_phase_ratio + 0.0 * (1 - left_phase_ratio))) < 0.01) {
      // from run to walk
      timer_gait = 0.0;
      trans_time = 0.3;
      gait_cycle = 1.0;          // 0.9;//
      left_phase_ratio = 0.36;   // 0.4;//
      right_phase_ratio = 0.36;  // 0.4;//
      left_theta_offset = left_phase_ratio + 0.0 * (1 - left_phase_ratio);
      right_theta_offset = left_theta_offset + 0.5;
      right_theta_offset = right_theta_offset - floor(right_theta_offset);
      gait_a = GaitEnum::Walk;
      std::cout << "change to walk!" << std::endl;
    } else if (gait_d == GaitEnum::Run && abs(left_phase) < 0.01) {
      // from walk to run
      timer_gait = 0.0;
      trans_time = 0.2;
      gait_cycle = 0.64;
      left_phase_ratio = 0.6;
      right_phase_ratio = 0.6;
      left_theta_offset = 0.0;
      right_theta_offset = left_theta_offset + 0.5;
      right_theta_offset = right_theta_offset - floor(right_theta_offset);
      gait_a = GaitEnum::Run;
      std::cout << "change to run!" << std::endl;
    }
  }

  if ((int)(timer_ / kDt) % freq_ == 0) {
    torch::Tensor output_data = mlp_model_->forward(inputs).toTensor();
    std::vector<float> out(output_data.data_ptr<float>(), output_data.data_ptr<float>() + output_data.numel());
    for (int i = 0; i < kObsDof; i++) {
      if (gait_a == GaitEnum::Stand) {
        output_data_mlp(i) = out[i];
      } else if (gait_a == GaitEnum::Walk) {
        output_data_mlp(i) = out[i + kObsDof];
        para_0 = last_action_d;
        para_1 = last_action_dot_d;
        para_2 = 3.0 * (output_data_mlp - last_action_d - last_action_dot_d * predictive_time) / predictive_time /
                     predictive_time -
                 (-last_action_dot_d) / predictive_time;
        para_3 = -2.0 * (output_data_mlp - last_action_d - last_action_dot_d * predictive_time) / predictive_time /
                     predictive_time / predictive_time +
                 (-last_action_dot_d) / predictive_time / predictive_time;
        timer_plan = 0.0;
      } else if (gait_a == GaitEnum::Run) {
        output_data_mlp(i) = out[i + kObsDof * 2];
        para_0 = last_action_d;
        para_1 = last_action_dot_d;
        para_2 = 3.0 * (output_data_mlp - last_action_d - last_action_dot_d * predictive_time) / predictive_time /
                     predictive_time -
                 (-last_action_dot_d) / predictive_time;
        para_3 = -2.0 * (output_data_mlp - last_action_d - last_action_dot_d * predictive_time) / predictive_time /
                     predictive_time / predictive_time +
                 (-last_action_dot_d) / predictive_time / predictive_time;
        timer_plan = 0.0;
      }
    }
  }

  Eigen::VectorXd mlp_out = output_data_mlp;
  Eigen::VectorXd mlp_out_dot = Eigen::VectorXd::Zero(kObsDof);
  if (gait_a == GaitEnum::Walk) {
    timer_plan += kDt;
    mlp_out =
        para_0 + para_1 * timer_plan + para_2 * timer_plan * timer_plan + para_3 * timer_plan * timer_plan * timer_plan;
    mlp_out_dot = para_1 + 2.0 * para_2 * timer_plan + 3.0 * para_3 * timer_plan * timer_plan;
  }

  Eigen::VectorXd mlp_out_scaled = mlp_out * action_scales + def_dof_pos_;
  Eigen::VectorXd mlp_out_dot_scaled = mlp_out_dot * action_scales;

  // trans from last gait to new gait
  if (timer_ == 0.0) {
    joint_pos_last_gait = last_action_d * action_scales + zero_pos_;
    joint_vel_last_gait = last_action_dot_d * action_scales;
  } else if (timer_gait == 0.0) {
    joint_pos_last_gait = last_action_d * action_scales + def_dof_pos_;
    joint_vel_last_gait = last_action_dot_d * action_scales;
  }

  if (timer_gait < trans_time) {
    mlp_out_scaled = (1.0 - timer_gait / trans_time) * joint_pos_last_gait + timer_gait / trans_time * mlp_out_scaled;
    mlp_out_dot_scaled =
        (1.0 - timer_gait / trans_time) * joint_vel_last_gait + timer_gait / trans_time * mlp_out_dot_scaled;
  }

  // do not control ankle roll
  mlp_out_scaled(5) = robot_data_->q_a_(11);
  mlp_out_scaled(11) = robot_data_->q_a_(17);

  // do not control wrist_yaw.* for webots
  setInfoFromJointIds(Eigen::VectorXd::Zero(wrist_yaw_ids_.size()), wrist_yaw_ids_, robot_data_->q_d_);
  setInfoFromJointIds(Eigen::VectorXd::Zero(wrist_yaw_ids_.size()), wrist_yaw_ids_, robot_data_->q_dot_d_);

  // set des here
  setInfoFromJointIds(mlp_out_scaled, obs_joint_ids_, robot_data_->q_d_);
  robot_data_->q_dot_d_.setZero();
  // set joint vel feedforward in PRL
  double flag_swing_left = 1.0;
  double flag_swing_right = 1.0;
  // flag_swing_left = gaitClock(left_phase,left_phase_ratio,0.02);
  // flag_swing_right = gaitClock(right_phase,right_phase_ratio,0.02);
  robot_data_->q_dot_d_(6) = mlp_out_dot_scaled(0) * flag_swing_left;
  robot_data_->q_dot_d_(7) = mlp_out_dot_scaled(1) * flag_swing_left;
  // robot_data_->q_dot_d_(8) = mlp_out_dot_scaled(2) * flag_swing_left;
  robot_data_->q_dot_d_(9) = mlp_out_dot_scaled(3) * flag_swing_left;
  robot_data_->q_dot_d_(12) = mlp_out_dot_scaled(6) * flag_swing_right;
  robot_data_->q_dot_d_(13) = mlp_out_dot_scaled(7) * flag_swing_right;
  // robot_data_->q_dot_d_(14) = mlp_out_dot_scaled(8) * flag_swing_right;
  robot_data_->q_dot_d_(15) = mlp_out_dot_scaled(9) * flag_swing_right;
  robot_data_->tau_d_.setZero();
  robot_data_->pos_mode_ = false;

  // std::cout << "robot_data_->q_d_:" << robot_data_->q_d_.transpose() << std::endl;

  last_action_d = mlp_out;
  last_action_dot_d = mlp_out_dot;
  action_last = output_data_mlp;
  timer_ += kDt;
  timer_gait += kDt;

#ifdef DATALOG
  DataHandler::getInstance().cacheData(robot_data_, timer_, output_data_mlp, input_data_mlp, gait_map[gait_a]);
#endif
}

FSMStateName StateMLP::checkTransition() {
  if (JsHum::getInst().getStateChange() == "gotoStop") {
    std::cout << "MLP2Stop" << std::endl;
    return FSMStateName::STOP;
  } else {
    return FSMStateName::MLP;
  }
}

void StateMLP::onExit() {}
