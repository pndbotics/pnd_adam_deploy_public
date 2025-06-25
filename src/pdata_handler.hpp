#ifndef PND_DATA_HANDLER_H_
#define PND_DATA_HANDLER_H_

#include "broccoli/core/Time.hpp"
#include "pconfig.hpp"
#include "pnd_algorithm.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

extern Eigen::VectorXd imu2_data;

class DataHandler {
 public:
  static DataHandler& getInstance() {
    static DataHandler instance;
    return instance;
  }
  ~DataHandler() { logger_->flush(); }
  DataHandler(DataHandler const&) = delete;
  DataHandler& operator=(DataHandler const&) = delete;

  void init() {
    // spdlog::init_thread_pool(8190, 1);
    time_t currentTime = time(nullptr);
    char chCurrentTime[256];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S", localtime(&currentTime));
    std::string stCurrentTime = chCurrentTime;
    std::string model_pb = PConfig::getInst().modelPb();
    auto pt_name = model_pb.substr(model_pb.rfind('/') + 1, model_pb.rfind('.') - model_pb.rfind('/') - 1);
    std::string filename = stCurrentTime + "_" + pt_name + "_log.txt";
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, 1024 * 1024 * 100, 3);
    rotating_sink->set_pattern("%v");
    std::vector<spdlog::sink_ptr> sinks{rotating_sink};
    logger_ = std::make_shared<spdlog::async_logger>("loggername", sinks.begin(), sinks.end(), spdlog::thread_pool(),
                                                     spdlog::async_overflow_policy::block);
    spdlog::register_logger(logger_);
  }

  void cacheData(RobotData& robot_data, double timeFSM, const broccoli::core::Time& get_state_time,
                 const broccoli::core::Time& fsm_time, const broccoli::core::Time& cmd_time,
                 const broccoli::core::Time& start_time, const broccoli::core::Time& total_time,
                 const broccoli::core::Time& timer, int cur_state) {
    // robot_data.dataL(0) = timeFSM;
    // robot_data.dataL(1) = get_state_time.m_nanoSeconds * 1e-6;
    // robot_data.dataL(2) = fsm_time.m_nanoSeconds * 1e-6;
    // robot_data.dataL(3) = cmd_time.m_nanoSeconds * 1e-6;
    // robot_data.dataL(4) = (timer.currentTime() - start_time).m_nanoSeconds * 1e-6;
    // robot_data.dataL(5) = total_time.m_nanoSeconds * 1e-6;
    // robot_data.dataL(6) = cur_state;
    robot_data.dataL.segment(0, 9) = imu2_data;
    robot_data.dataL.segment(9, 9) = robot_data.imu_data_;
  }

  void cacheData(RobotData* robot_data, double timer) {
    robot_data->dataL[start_pos_] = timer;
    robot_data->dataL.block(start_pos_ + 1, 0, kRobotDataSize, 1) = robot_data->q_d_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize, 0, kRobotDataSize, 1) = robot_data->q_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 2, 0, kRobotDataSize, 1) = robot_data->q_dot_d_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 3, 0, kRobotDataSize, 1) = robot_data->q_dot_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 4, 0, kRobotDataSize, 1) = robot_data->tau_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 5, 0, kRobotDataSize, 1) = robot_data->tau_d_;
  }

  void cacheData(RobotData* robot_data, double timer, const Eigen::VectorXd& output_data_mlp,
                 const Eigen::VectorXd& input_data_mlp, double gait_a) {
    robot_data->dataL[start_pos_] = timer;
    robot_data->dataL.block(start_pos_ + 1, 0, kRobotDataSize, 1) = robot_data->q_d_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize, 0, kRobotDataSize, 1) = robot_data->q_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 2, 0, kRobotDataSize, 1) = robot_data->q_dot_d_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 3, 0, kRobotDataSize, 1) = robot_data->q_dot_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 4, 0, kRobotDataSize, 1) = robot_data->tau_a_;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 5, 0, kObsDof, 1) = output_data_mlp;
    robot_data->dataL.block(start_pos_ + kRobotDataSize * 5 + kObsDof, 0, PConfig::getInst().obsNum(), 1) =
        input_data_mlp;
    robot_data->dataL[start_pos_ + kRobotDataSize * 5 + kObsDof + PConfig::getInst().obsNum()] = gait_a;
  }

  void writeData(const RobotData& robot_data) {
    for (const auto& elem : robot_data.dataL) {
      oss_ << elem << " ";
    }
    logger_->info(oss_.str());
    oss_.str("");
  }

 private:
  DataHandler() = default;

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::async_logger> logger_;

  int start_pos_ = 50;
};

#endif  // PND_DATA_HANDLER_H_