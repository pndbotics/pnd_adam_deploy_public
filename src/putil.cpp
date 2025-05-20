#include "putil.h"

void getInfoFormJointIds(const Eigen::VectorXd& joint_info, const std::vector<int>& joint_ids, Eigen::VectorXd& info) {
  for (size_t i = 0; i < joint_ids.size(); i++) {
    info[i] = joint_info[joint_ids[i] + kBaseNum];
  }
}

void setInfoFromJointIds(const Eigen::VectorXd& info, const std::vector<int>& joint_ids, Eigen::VectorXd& joint_info) {
  for (size_t i = 0; i < joint_ids.size(); i++) {
    joint_info[joint_ids[i] + kBaseNum] = info[i];
  }
}

double radToDeg(double radians) { return radians * (180.0 / M_PI); }

void incrementLastField(std::vector<std::string>& keys) {
  for (auto& key : keys) {
    std::istringstream iss(key);
    std::string token;
    std::vector<std::string> parts;

    while (std::getline(iss, token, '.')) {
      parts.push_back(token);
    }

    int lastField = std::stoi(parts.back());
    lastField += 10;

    parts.back() = std::to_string(lastField);

    std::ostringstream oss;
    for (size_t i = 0; i < parts.size(); ++i) {
      oss << parts[i];
      if (i < parts.size() - 1) {
        oss << '.';
      }
    }

    key = oss.str();
  }
}
