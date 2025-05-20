#include "basic_function.h"

Eigen::Matrix3d rotX(double x) { return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d rotY(double y) { return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d rotZ(double z) { return Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

void eulerXYZToMatrix(Eigen::Matrix3d &R, Eigen::Vector3d euler_a) {
  R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void clip(Eigen::VectorXd &in_, double lb, double ub) {
  for (int i = 0; i < in_.size(); i++) {
    if (in_[i] < lb) {
      in_[i] = lb;
    }
    if (in_[i] > ub) {
      in_[i] = ub;
    }
  }
}

double clip(double a, double lb, double ub) {
  double b = a;
  if (a < lb) {
    b = lb;
  }
  if (a > ub) {
    b = ub;
  }
  return b;
}

Eigen::VectorXd gaitPhase(double timer, double gait_cycle_, double left_theta_offset_, double right_theta_offset_,
                          double left_phase_ratio_, double right_phase_ratio_) {
  Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
  double left_phase = (timer / gait_cycle_ + left_theta_offset_) - floor(timer / gait_cycle_ + left_theta_offset_);
  double right_phase = (timer / gait_cycle_ + right_theta_offset_) - floor(timer / gait_cycle_ + right_theta_offset_);
  res(0) = sin(2.0 * M_PI * left_phase);
  res(1) = sin(2.0 * M_PI * right_phase);
  res(2) = cos(2.0 * M_PI * left_phase);
  res(3) = cos(2.0 * M_PI * right_phase);
  res(4) = left_phase_ratio_;
  res(5) = right_phase_ratio_;
  return res;
}

double gaitClock(double phase_, double ratio_, double t_) {
  if (phase_ < t_) {
    return (0.5 + phase_ / (2.0 * t_));
  } else if ((phase_ >= t_) && (phase_ <= ratio_ - t_)) {
    return 1.0;
  } else if ((phase_ > (ratio_ - t_)) & (phase_ < (ratio_ + t_))) {
    return (phase_ - ratio_ - t_) / (2.0 * t_);
  } else if ((phase_ >= (ratio_ + t_)) & (phase_ <= (1.0 - t_))) {
    return 0.0;
  } else {
    return (phase_ - 1.0 + t_) / (2.0 * t_);
  }
}

void fifthPoly(const Eigen::VectorXd &p0, const Eigen::VectorXd &p0_dot, const Eigen::VectorXd &p0_dotdot,  //
               const Eigen::VectorXd &p1, const Eigen::VectorXd &p1_dot, const Eigen::VectorXd &p1_dotdot,  //
               double totalTime,    // total time
               double currentTime,  // current time,from 0 to total time
               Eigen::VectorXd &pd, Eigen::VectorXd &pd_dot, Eigen::VectorXd &pd_dotdot) {
  double t = currentTime;
  double time = totalTime;
  if (t < totalTime) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,       //
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,        //
        0.0, 0.0, 1.0 / 2.0, 0.0, 0.0, 0.0,  //
        -10 / pow(time, 3), -6 / pow(time, 2), -3 / (2 * time), 10 / pow(time, 3), -4 / pow(time, 2),
        1 / (2 * time),  //
        15 / pow(time, 4), 8 / pow(time, 3), 3 / (2 * pow(time, 2)), -15 / pow(time, 4), 7 / pow(time, 3),
        -1 / pow(time, 2),  //
        -6 / pow(time, 5), -3 / pow(time, 4), -1 / (2 * pow(time, 3)), 6 / pow(time, 5), -3 / pow(time, 4),
        1 / (2 * pow(time, 3));
    Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 1);
    for (int i = 0; i < p0.size(); i++) {
      x0 << p0(i), p0_dot(i), p0_dotdot(i), p1(i), p1_dot(i), p1_dotdot(i);
      a = A * x0;
      pd(i) = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t;
      pd_dot(i) = a(1) + 2 * a(2) * t + 3 * a(3) * t * t + 4 * a(4) * t * t * t + 5 * a(5) * t * t * t * t;
      pd_dotdot(i) = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t * t + 20 * a(5) * t * t * t;
    }
  } else {
    pd = p1;
    pd_dot = p1_dot;
    pd_dotdot = p1_dotdot;
  }
}

LowPassFilter::LowPassFilter(double cutOffFreq, double dampRatio, double dTime, int nFilter) {
  dt_ = dTime;
  sig_in_1_ = Eigen::VectorXd::Zero(nFilter);
  sig_in_2_ = Eigen::VectorXd::Zero(nFilter);
  sig_out_1_ = Eigen::VectorXd::Zero(nFilter);
  sig_out_2_ = Eigen::VectorXd::Zero(nFilter);

  double freqInRad = 2. * M_PI * cutOffFreq;
  double c = 2.0 / dt_;
  double sqrC = c * c;
  double sqrW = freqInRad * freqInRad;

  b2_ = sqrC + 2.0 * dampRatio * freqInRad * c + sqrW;
  b1_ = -2.0 * (sqrC - sqrW);
  b0_ = sqrC - 2.0 * dampRatio * freqInRad * c + sqrW;

  a2_ = sqrW;
  a1_ = 2.0 * sqrW;
  a0_ = sqrW;
  a2_ /= b2_;
  a1_ /= b2_;
  a0_ /= b2_;

  b1_ /= b2_;
  b0_ /= b2_;
  b2_ = 1.0;
}

Eigen::VectorXd LowPassFilter::mFilter(Eigen::VectorXd sigIn) {
  Eigen::VectorXd sigOut = a2_ * sigIn + a1_ * sig_in_1_ + a0_ * sig_in_2_ - b1_ * sig_out_1_ - b0_ * sig_out_2_;
  sig_in_2_ = sig_in_1_;
  sig_in_1_ = sigIn;
  sig_out_2_ = sig_out_1_;
  sig_out_1_ = sigOut;
  return sigOut;
}
