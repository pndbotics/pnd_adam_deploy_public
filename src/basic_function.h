

#ifndef BASICFUNCTION_H_
#define BASICFUNCTION_H_

#include <Eigen/Dense>

Eigen::Matrix3d rotX(double x);
Eigen::Matrix3d rotY(double y);
Eigen::Matrix3d rotZ(double z);
void eulerXYZToMatrix(Eigen::Matrix3d& R, Eigen::Vector3d euler_a);
void clip(Eigen::VectorXd& in_, double lb, double ub);
double clip(double a, double lb, double ub);
Eigen::VectorXd gaitPhase(double timer, double gait_cycle_, double left_theta_offset_, double right_theta_offset_,
                          double left_phase_ratio_, double right_phase_ratio_);
double gaitClock(double phase_, double ratio_, double t_);

void fifthPoly(const Eigen::VectorXd& p0, const Eigen::VectorXd& p0_dot, const Eigen::VectorXd& p0_dotdot,  // start
               const Eigen::VectorXd& p1, const Eigen::VectorXd& p1_dot, const Eigen::VectorXd& p1_dotdot,  // end
               double totalTime,    // total time
               double currentTime,  // current time,from 0 to total time
               Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot, Eigen::VectorXd& pd_dotdot);  // output command

class LowPassFilter {
 public:
  LowPassFilter(double cutOffFreq, double dampRatio, double dTime, int nFilter);
  Eigen::VectorXd mFilter(Eigen::VectorXd sigIn);

 private:
  double dt_;
  Eigen::VectorXd sig_in_1_;
  Eigen::VectorXd sig_in_2_;
  Eigen::VectorXd sig_out_1_;
  Eigen::VectorXd sig_out_2_;
  double b2_, b1_, b0_;
  double a2_, a1_, a0_;
};

#endif  // BASICFUNCTION_H_
