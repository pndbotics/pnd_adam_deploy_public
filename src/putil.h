#ifndef PUTIL_H_
#define PUTIL_H_

#include <Eigen/Dense>

#include "pconfig.hpp"
#include "pnd_algorithm.h"

void getInfoFormJointIds(const Eigen::VectorXd& joint_info, const std::vector<int>& joint_ids, Eigen::VectorXd& info);

void setInfoFromJointIds(const Eigen::VectorXd& info, const std::vector<int>& joint_ids, Eigen::VectorXd& joint_info);

double radToDeg(double radians);

void incrementLastField(std::vector<std::string>& keys);

#endif