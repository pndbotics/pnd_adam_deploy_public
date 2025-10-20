#ifndef PCONFIG_HPP_
#define PCONFIG_HPP_

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "nlohmann/json/json.hpp"
#include "pnd_algorithm.h"
#include "yaml-cpp/yaml.h"

struct PCfg_ {
  // joint config
  std::string ip;
  std::string name;
  int joint_dir;
  double joint_gear_ratio;
  double c_t_scale;
  double zero_pos;
  double default_dof_pos;
  double kd_scale;
  double joint_max_current;

  // absolute encoder config
  float abs_pos_gear_ratio;
  float abs_pos_zero;
  float abs_pos_dir;
  float motor_rotor_abs_pos;

  PCfg_(std::string ip, std::string name, int joint_dir, double joint_gear_ratio, double c_t_scale, double zero_pos,
        double default_dof_pos, double kd_scale, double joint_max_current, float abs_pos_gear_ratio, float abs_pos_zero,
        float abs_pos_dir)
      : ip(ip),
        name(name),
        joint_dir(joint_dir),
        joint_gear_ratio(joint_gear_ratio),
        c_t_scale(c_t_scale),
        zero_pos(zero_pos),
        default_dof_pos(default_dof_pos),
        kd_scale(kd_scale),
        joint_max_current(joint_max_current),
        abs_pos_gear_ratio(abs_pos_gear_ratio),
        abs_pos_zero(abs_pos_zero),
        abs_pos_dir(abs_pos_dir) {}
};

class PConfig {
 public:
  static PConfig& getInst() {
    static PConfig instance;
    return instance;
  }
  ~PConfig() = default;
  PConfig(PConfig const&) = delete;
  PConfig& operator=(PConfig const&) = delete;

  void loadConfig() {
    try {
      config_ = YAML::LoadFile("config.yaml");
    } catch (const YAML::BadFile& e) {
      config_ = YAML::LoadFile("../config.yaml");
    }
    model_pb_ = config_["model_pb"].as<std::string>();
    obs_num_ = config_["obs_num"].as<int>();
    state_name_ = config_["state_name"].as<std::string>();
    joint_config_path_ = config_["joint_config_path"].as<std::string>();
    validateFilePath(model_pb_);
    validateFilePath(joint_config_path_);

    // read joint abs config json
    std::ifstream joint_abs_config_file("/root/.adam/joint_abs_config.json");
    nlohmann::json abs_data;
    if (!joint_abs_config_file.is_open()) {
      std::cout << "/root/.adam/joint_abs_config.json not found" << std::endl;
    }
    try {
      abs_data = nlohmann::json::parse(joint_abs_config_file);
    } catch (std::exception& e) {
      std::cout << "joint_abs_config.json parse error" << std::endl;
      std::cout << e.what() << std::endl;
    }
    joint_abs_config_file.close();
    auto joint_names = PConfig::getInst().jointNames();
    for (auto& item : abs_data.items()) {
      auto it = std::find(joint_names.begin(), joint_names.end(), item.key());
      if (it != joint_names.end()) {
        int idx = std::distance(joint_names.begin(), it);
        pcfg_[idx].abs_pos_zero = item.value()["absolute_pos_zero"].get<float>();
        pcfg_[idx].abs_pos_gear_ratio = item.value()["absolute_pos_gear_ratio"].get<float>();
        pcfg_[idx].abs_pos_dir = item.value()["absolute_pos_dir"].get<float>();
        try {
          pcfg_[idx].motor_rotor_abs_pos = item.value()["motor_rotor_abs_pos"].get<float>();
        } catch (std::exception& e) {
          std::cout << "WARNING: joint" << item.key() << " motor_rotor_abs_pos not set in config file." << std::endl;
          pcfg_[idx].motor_rotor_abs_pos = 0;
        }
      } else {
        std::cout << "ERROR: joint " << item.key() << " abs not found" << std::endl;
      }
    }
  }

  void validateFilePath(const std::string& file_path) {
    if (file_path.empty()) {
      std::cout << "file_path is empty" << file_path << std::endl;
      exit(-1);
    }
    std::ifstream file(file_path);
    if (!file.good()) {
      std::cout << "File does not exist: " << file_path << std::endl;
      exit(-1);
    }
  }

  const std::string modelPb() const { return model_pb_; }
  const int obsNum() const { return obs_num_; }
  const std::string stateName() const { return state_name_; }
  const std::string jointConfigPath() const { return joint_config_path_; }

  Eigen::VectorXd absolutePosGearRatio() const {
    Eigen::VectorXd abs_pos_gear_ratio = Eigen::VectorXd::Zero(pcfg_.size());
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      abs_pos_gear_ratio[i] = pcfg_[i].abs_pos_gear_ratio;
    }
    return abs_pos_gear_ratio;
  }
  Eigen::VectorXd absolutePosZero() const {
    int count = 0;
    Eigen::VectorXd abs_pos_zero = Eigen::VectorXd::Zero(pcfg_.size());
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      abs_pos_zero[i] = pcfg_[i].abs_pos_zero;
      if (abs_pos_zero[i] == 0) {
        ++count;
      }
    }

    if (count >= kRobotDof) {
      std::cout << "abs_pos_zero read failed." << std::endl;
      exit(-1);
    }
    return abs_pos_zero;
  }
  Eigen::VectorXd absolutePosDir() const {
    Eigen::VectorXd abs_pos_dir = Eigen::VectorXd::Zero(pcfg_.size());
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      abs_pos_dir[i] = pcfg_[i].abs_pos_dir;
    }
    return abs_pos_dir;
  }
  Eigen::VectorXd motorRotorAbsPos() const {
    Eigen::VectorXd motor_rotor_abs_pos = Eigen::VectorXd::Zero(pcfg_.size());
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      motor_rotor_abs_pos[i] = pcfg_[i].motor_rotor_abs_pos;
    }
    return motor_rotor_abs_pos;
  }

  std::vector<std::string> jointNames() {
    std::vector<std::string> joint_names;
    for (size_t i = 0; i < pcfg_.size(); i++) {
      joint_names.push_back(pcfg_[i].name);
    }
    return joint_names;
  }

  const Eigen::VectorXd& zeroPos() const { return zero_pos_; }
  Eigen::VectorXd zeroPosFromIds(const std::vector<int>& joint_ids) const {
    Eigen::VectorXd zero_pos = Eigen::VectorXd::Zero(joint_ids.size());
    for (size_t i = 0; i < joint_ids.size(); i++) {
      zero_pos[i] = zero_pos_[joint_ids[i]];
    }
    return zero_pos;
  }

  const Eigen::VectorXd& defaultDofPos() const { return default_dof_pos_; }
  Eigen::VectorXd defDofPosFromIds(const std::vector<int>& joint_ids) const {
    Eigen::VectorXd def_dof_pos = Eigen::VectorXd::Zero(joint_ids.size());
    for (size_t i = 0; i < joint_ids.size(); i++) {
      def_dof_pos[i] = default_dof_pos_[joint_ids[i]];
    }
    return def_dof_pos;
  }

  const Eigen::VectorXd& kdScale() const { return kd_scale_; }
  Eigen::VectorXd jointMaxCurrent() const {
    Eigen::VectorXd joint_max_current = Eigen::VectorXd::Zero(pcfg_.size());
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      joint_max_current[i] = pcfg_[i].joint_max_current;
    }
    return joint_max_current;
  }

  std::vector<std::string> ipList() const {
    std::vector<std::string> ip_list;
    for (size_t i = 0; i < pcfg_.size(); ++i) {
      ip_list.push_back(pcfg_[i].ip);
    }
    return ip_list;
  }
  const Eigen::VectorXi& jointDir() const { return joint_dir_; }
  const Eigen::VectorXd& jointGearRatio() const { return joint_gear_ratio_; }
  const Eigen::VectorXd& curTorScale() const { return c_t_scale_; }
  std::vector<int> jointIdFromNames(const std::vector<std::string>& joint_names) const {
    std::vector<int> joint_ids;
    for (auto& name : joint_names) {
      auto it = joint_name_idx_map_.find(name);
      if (it != joint_name_idx_map_.end()) {
        joint_ids.push_back(it->second);
      } else {
        joint_ids.push_back(-1);
        std::cout << "ERROR: joint " << name << " not found" << std::endl;
      }
    }
    return joint_ids;
  }

  const YAML::Node config() const {
    if (config_[state_name_]) {
      return config_[state_name_];
    } else {
      std::cout << "ERROR: config for state " << state_name_ << " not found" << std::endl;
      return YAML::Node();
    }
  }

 private:
  void addCfg(std::string ip, std::string name, int joint_dir, double joint_gear_ratio, double c_t_scale,
              double zero_pos, double default_dof_pos, double kd_scale, double joint_max_current,
              float abs_pos_gear_ratio, float abs_pos_zero, float abs_pos_dir) {
    PCfg_ cfg(ip, name, joint_dir, joint_gear_ratio, c_t_scale, zero_pos, default_dof_pos, kd_scale, joint_max_current,
              abs_pos_gear_ratio, abs_pos_zero, abs_pos_dir);
    pcfg_.push_back(cfg);
  }

  PConfig() {
    // clang-format off
    addCfg("10.10.10.70", "hipPitch_Left",       -1,  7,  0.227, -0.41,  -0.586, 1.66, 110, 1, 0, 1);
    addCfg("10.10.10.71", "hipRoll_Left",        -1, 31,  0.136, -0.04,  -0.085, 20.0, 45,  1, 0, 1);
    addCfg("10.10.10.72", "hipYaw_Left",          1, 51,  0.074, -0.23,  -0.322, 30.1, 30,  1, 0, 1);
    addCfg("10.10.10.73", "kneePitch_Left",       1,  7,  0.227,  0.81,   1.288, 1.66, 110, 1, 0, 1);
    addCfg("10.10.10.74", "anklePitch_Left",     -1, 30, 0.0592, -0.47,  -0.789, 16.5, 30,  1, 0, 1);
    addCfg("10.10.10.75", "ankleRoll_Left",       1, 30, 0.0592,  0.0,    0.002, 16.5, 30,  1, 0, 1);
    addCfg("10.10.10.50", "hipPitch_Right",       1,  7,  0.227, -0.41,  -0.586, 1.66, 110, 1, 0, 1);
    addCfg("10.10.10.51", "hipRoll_Right",       -1, 31,  0.136,  0.04,   0.085, 20.0, 45,  1, 0, 1);
    addCfg("10.10.10.52", "hipYaw_Right",         1, 51,  0.074,  0.23,   0.322, 30.1, 30,  1, 0, 1);
    addCfg("10.10.10.53", "kneePitch_Right",     -1,  7,  0.227,  0.81,   1.288, 1.66, 110, 1, 0, 1);
    addCfg("10.10.10.54", "anklePitch_Right",     1, 30, 0.0592, -0.47,  -0.789, 16.5, 30,  1, 0, 1);
    addCfg("10.10.10.55", "ankleRoll_Right",     -1, 30, 0.0592,  0.0,   -0.002, 16.5, 30,  1, 0, 1);
    addCfg("10.10.10.90", "waistRoll",            1, 51,  0.074,  0.0,    0.0,   30.1, 30,  1, 0, 1);
    addCfg("10.10.10.91", "waistPitch",          -1, 51,  0.074,  0.0,    0.0,   30.1, 30,  1, 0, 1);
    addCfg("10.10.10.92", "waistYaw",            -1, 51,  0.074,  0.0,    0.0,   30.1, 30,  1, 0, 1);
#if defined(ADAM_SP_PRO)
    addCfg("10.10.10.93", "neckYaw",             -1, 51,  0.063,  0.0,    0.0,   19.8, 6,   1, 0, 1);
    addCfg("10.10.10.94", "neckPitch",           -1, 51,  0.063,  0.0,    0.0,   19.8, 6,   1, 0, 1);
#endif
    addCfg("10.10.10.10", "shoulderPitch_Left",  -1, 51, 0.0592,  0.0,    0.0,   8.25, 30,  1, 0, 1);
    addCfg("10.10.10.11", "shoulderRoll_Left",    1, 51, 0.0592,  0.0,    0.0,   8.25, 30,  1, 0, 1);
    addCfg("10.10.10.12", "shoulderYaw_Left",     1, 51,  0.063,  0.0,    0.0,   19.8, 6,   1, 0, 1);
    addCfg("10.10.10.13", "elbow_Left",          -1, 51,  0.063, -0.3,   -0.3,   19.8, 6,   1, 0, 1);
    addCfg("10.10.10.14", "wristYaw_Left",        1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#if defined(ADAM_STANDARD) || defined(ADAM_INSPIRE) || defined(ADAM_SP_PRO)
    addCfg("10.10.10.15", "wristPitch_Left",     -1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
    addCfg("10.10.10.16", "wristRoll_Left",       1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#endif
#ifdef ADAM_STANDARD
    addCfg("10.10.10.17", "gripper_Left",         1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#endif
    addCfg("10.10.10.30", "shoulderPitch_Right",  1, 51, 0.0592,  0.0,    0.0,   8.25, 30,  1, 0, 1);
    addCfg("10.10.10.31", "shoulderRoll_Right",   1, 51, 0.0592,  0.0,    0.0,   8.25, 30,  1, 0, 1);
    addCfg("10.10.10.32", "shoulderYaw_Right",    1, 51,  0.063,  0.0,    0.0,   19.8, 6,   1, 0, 1);
    addCfg("10.10.10.33", "elbow_Right",          1, 51,  0.063, -0.3,   -0.3,   19.8, 6,   1, 0, 1);
    addCfg("10.10.10.34", "wristYaw_Right",       1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#if defined(ADAM_STANDARD) || defined(ADAM_INSPIRE) || defined(ADAM_SP_PRO)
    addCfg("10.10.10.35", "wristPitch_Right",     1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
    addCfg("10.10.10.36", "wristRoll_Right",     -1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#endif
#ifdef ADAM_STANDARD
    addCfg("10.10.10.37", "gripper_Right",        1, 51,  0.063,  0.0,    0.0,   19.8, 4,   1, 0, 1);
#endif
    // clang-format on

    zero_pos_ = Eigen::VectorXd::Zero(kRobotDof);
    default_dof_pos_ = Eigen::VectorXd::Zero(kRobotDof);
    kd_scale_ = Eigen::VectorXd::Zero(kRobotDof);
    joint_dir_ = Eigen::VectorXi::Zero(kRobotDof);
    joint_gear_ratio_ = Eigen::VectorXd::Zero(kRobotDof);
    c_t_scale_ = Eigen::VectorXd::Zero(kRobotDof);

    for (size_t i = 0; i < pcfg_.size(); ++i) {
      joint_name_idx_map_[pcfg_[i].name] = i;

      zero_pos_[i] = pcfg_[i].zero_pos;
      default_dof_pos_[i] = pcfg_[i].default_dof_pos;
      kd_scale_[i] = pcfg_[i].kd_scale;
      joint_dir_[i] = pcfg_[i].joint_dir;
      joint_gear_ratio_[i] = pcfg_[i].joint_gear_ratio;
      c_t_scale_[i] = pcfg_[i].c_t_scale;

      // kd_scale_[i] = c_t_scale_[i] * joint_gear_ratio_[i] *joint_gear_ratio_[i] /(2 * M_PI);
    }

    std::vector<std::string> ankle_names = {
        "anklePitch_Right",
        "ankleRoll_Right",
        "anklePitch_Left",
        "ankleRoll_Left",
    };
    auto ankle_idx = jointIdFromNames(ankle_names);
    for (auto& idx : ankle_idx) {
      kd_scale_[idx] = c_t_scale_[idx] * joint_gear_ratio_[idx] * joint_gear_ratio_[idx] / (2 * M_PI);
    }
  };

 private:
  YAML::Node config_;

  std::string model_pb_;
  int obs_num_;
  std::string state_name_;
  std::string joint_config_path_;
  std::map<std::string, int> joint_name_idx_map_;
  std::vector<PCfg_> pcfg_;

  Eigen::VectorXd zero_pos_;
  Eigen::VectorXd default_dof_pos_;
  Eigen::VectorXd kd_scale_;
  Eigen::VectorXi joint_dir_;
  Eigen::VectorXd joint_gear_ratio_;
  Eigen::VectorXd c_t_scale_;
};

#endif

// walk
// zero_pos_ << -0.586, -0.085, -0.322, 1.288, -0.789, 0.002,
//     -0.586, 0.085, 0.322, 1.288, -0.789, -0.002,
//     0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, -0.3, 0.0, -0.0, 0.0, -0.3;
// qdd
//  kd_scale_ << 15.99, 21.91, 1.77, 1.85, 31.25, 31.25,
//              15.99, 22.29, 1.73, 1.85, 31.25, 31.25,
//              21.19, 22.30, 21.50,
//              19.80, 19.80, 19.80, 19.80,
//              19.80, 19.80, 19.80, 19.80;
// harm
//  kd_scale_ << 1.77, 15.99, 21.91, 1.85, 39.6, 39.6,
//      1.73, 15.99, 22.29, 1.85, 39.6, 39.6,
//      21.19, 22.30, 21.50,
//      19.80, 19.80, 19.80, 19.80,
//      19.80, 19.80, 19.80, 19.80;