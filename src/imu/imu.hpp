
#pragma once

#include <Eigen/Dense>
#include <memory>

enum class ImuType { VN100_IMU, CH_IMU, UNKNOWN };

ImuType checkImuType();

class ImuInterface {
 public:
  virtual ~ImuInterface() = default;
  virtual bool initialize() = 0;
  virtual const Eigen::VectorXd& getImuData() = 0;
  virtual bool close() { return true; }
};

class ImuHandler {
 public:
  ImuHandler() = default;
  ~ImuHandler() = default;
  bool initialize();
  const Eigen::VectorXd& getImuData() const { return imu_interface->getImuData(); }
  bool close() {
    if (imu_interface) {
      return imu_interface->close();
    }
    return true;
  }

 private:
  std::shared_ptr<ImuInterface> imu_interface = nullptr;
};
