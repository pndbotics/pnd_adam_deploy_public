#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <imu.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace LibSerial;

// List of baud rates to try
const std::vector<LibSerial::BaudRate> BAUD_RATES = {LibSerial::BaudRate::BAUD_115200,
                                                     LibSerial::BaudRate::BAUD_921600};

// Function to calculate checksum
std::string calc_checksum(const std::string& cmd) {
  unsigned char checksum = 0;
  for (char c : cmd) {
    checksum ^= c;
  }
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)checksum;
  return oss.str();
}

// Function to read multiple lines from serial port
std::vector<std::string> read_serial_lines(SerialPort& ser, size_t max_lines = 10, double timeout = 0.1) {
  std::vector<std::string> lines;
  auto start_time = std::chrono::steady_clock::now();

  ser.FlushInputBuffer();

  while (lines.size() < max_lines &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < timeout) {
    std::string line;
    try {
      ser.ReadLine(line, '\n', timeout * 1000);
      if (!line.empty()) {
        // Remove trailing whitespace and CR/LF
        line.erase(std::remove_if(line.begin(), line.end(), [](char c) { return c == '\r' || c == '\n'; }), line.end());
        lines.push_back(line);
      }
    } catch (const ReadTimeout&) {
      // Timeout occurred while reading
      break;
    } catch (const std::exception& e) {
      std::cerr << "[ERROR] Serial read exception: " << e.what() << std::endl;
      break;
    }
  }

  return lines;
}

void vn100_send_command(SerialPort& ser, const std::string& base_cmd) {
  std::string full_cmd = "$" + base_cmd + "*" + calc_checksum(base_cmd) + "\r\n";
  ser.Write(full_cmd);
}

// Function to send VN100 request
std::string vn100_request(SerialPort& ser, const std::string& base_cmd) {
  vn100_send_command(ser, base_cmd);

  auto lines = read_serial_lines(ser, 10, 0.1);

  if (lines.empty()) {
    return "";
  }

  for (const auto& line : lines) {
    if (line.find("$" + base_cmd) == 0) {
      size_t first_comma = line.find(',');
      if (first_comma != std::string::npos) {
        size_t second_comma = line.find(',', first_comma + 1);
        if (second_comma != std::string::npos) {
          size_t star_pos = line.find('*', second_comma + 1);
          if (star_pos != std::string::npos) {
            return line.substr(second_comma + 1, star_pos - second_comma - 1);
          }
        }
      }
    }
  }

  return "";
}

// Function to send CH request
std::map<std::string, std::string> ch_request(SerialPort& ser) {
  std::map<std::string, std::string> data;

  ser.Write("LOG DISABLE\r\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ser.Write("LOG VERSION\r\n");

  auto lines = read_serial_lines(ser, 10, 0.1);

  if (lines.size() == 6) {
    for (const auto& item : lines) {
      size_t eq_pos = item.find('=');
      if (eq_pos != std::string::npos) {
        std::string key = item.substr(0, eq_pos);
        std::string value = item.substr(eq_pos + 1);
        data[key] = value;
      } else {
        data["status"] = item;
      }
    }
  }

  return data;
}

// Function to check CH IMU
std::map<std::string, std::string> ch_check(SerialPort& ser) {
  std::map<std::string, std::string> data;
  data["model"] = "PND_IMU_02";

  auto value = ch_request(ser);
  if (value.find("UUID") != value.end()) {
    data["sn"] = value["UUID"];
    return data;
  }

  return {};
}

// Function to check VN100 IMU
std::map<std::string, std::string> vn100_check(SerialPort& ser) {
  std::map<std::string, std::string> data;
  data["model"] = "PND_IMU_01";

  vn100_send_command(ser, "VNASY,0");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::string value = vn100_request(ser, "VNRRG,03");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  vn100_send_command(ser, "VNASY,1");
  if (!value.empty()) {
    data["sn"] = value;
    return data;
  }

  return {};
}

// Function to scan a serial port
std::map<std::string, std::string> scan_serial(SerialPort& ser) {
  try {
    auto data = vn100_check(ser);
    if (!data.empty()) {
      return data;
    }

    data = ch_check(ser);
    if (!data.empty()) {
      return data;
    }
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] Test failed: " << e.what() << std::endl;
  }

  return {};
}

// Custom sort for USB devices
bool usb_sort(const std::string& a, const std::string& b) {
  std::regex re("\\d+");
  std::smatch match_a, match_b;

  std::regex_search(a, match_a, re);
  std::regex_search(b, match_b, re);

  if (match_a.empty() || match_b.empty()) {
    return a < b;
  }

  int num_a = std::stoi(match_a[0]);
  int num_b = std::stoi(match_b[0]);

  return num_a < num_b;
}

// Main scanning function
std::map<std::string, std::map<std::string, std::string>> imu_scan() {
  std::map<std::string, std::map<std::string, std::string>> results;

  // Get all /dev/ttyUSB* devices
  std::vector<std::string> ports;
  for (const auto& entry : fs::directory_iterator("/dev")) {
    std::string path = entry.path();
    if (path.find("/dev/ttyUSB") == 0) {
      ports.push_back(path);
    }
  }

  if (ports.empty()) {
    std::cout << "[INFO] No /dev/ttyUSB* devices found" << std::endl;
    return results;
  }

  // Sort ports numerically
  std::sort(ports.begin(), ports.end(), usb_sort);

  std::cout << "[INFO] Found " << ports.size() << " devices: ";
  for (const auto& port : ports) {
    std::cout << port << " ";
  }
  std::cout << std::endl;

  for (const auto& device : ports) {
    std::cout << "\n[DEVICE] Scanning device: " << device << std::endl;

    for (auto baud : BAUD_RATES) {
      try {
        SerialPort ser;
        ser.Open(device);
        ser.SetBaudRate(baud);
        ser.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        ser.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        ser.SetParity(Parity::PARITY_NONE);
        ser.SetStopBits(StopBits::STOP_BITS_1);

        std::cout << "[OPEN] Successfully opened " << device << " @ " << static_cast<unsigned int>(baud) << "bps"
                  << std::endl;

        auto result = scan_serial(ser);
        if (!result.empty()) {
          results[device] = result;
          ser.Close();
          break;  // Success, move to next device
        }

        ser.Close();
      } catch (const OpenFailed&) {
        std::cerr << "[FAIL] " << device << " @ " << static_cast<unsigned int>(baud) << "bps failed to open"
                  << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << device << " @ " << static_cast<unsigned int>(baud) << "bps error: " << e.what()
                  << std::endl;
      }
    }
  }

  return results;
}

ImuType checkImuType() {
  auto results = imu_scan();
  ImuType imu_type = ImuType::UNKNOWN;
  if (results.empty()) {
    std::cerr << "[ERROR] No IMU devices found" << std::endl;
  } else if (results.begin()->second.find("model") != results.begin()->second.end() &&
             results.begin()->second["model"] == "PND_IMU_01") {
    std::cout << "[INFO] Detected VN100 IMU" << std::endl;
    imu_type = ImuType::VN100_IMU;
  } else if (results.begin()->second.find("model") != results.begin()->second.end() &&
             results.begin()->second["model"] == "PND_IMU_02") {
    std::cout << "[INFO] Detected CH IMU" << std::endl;
    imu_type = ImuType::CH_IMU;
  } else {
    std::cerr << "[ERROR] IMU type detection failed" << std::endl;
    std::cerr << "[ERROR] Please check the device connection and ensure it is a supported IMU." << std::endl;
    std::cerr << "[ERROR] If the device is supported, please report this issue to the developers." << std::endl;
  }
  return imu_type;
}

int main_test() {
  auto results = imu_scan();

  // Print results in JSON-like format
  std::cout << "{\n";
  for (auto it = results.begin(); it != results.end(); ++it) {
    std::cout << "  \"" << it->first << "\": {\n";
    for (auto inner_it = it->second.begin(); inner_it != it->second.end(); ++inner_it) {
      std::cout << "    \"" << inner_it->first << "\": \"" << inner_it->second << "\"";
      if (std::next(inner_it) != it->second.end()) {
        std::cout << ",";
      }
      std::cout << "\n";
    }
    std::cout << "  }";
    if (std::next(it) != results.end()) {
      std::cout << ",";
    }
    std::cout << "\n";
  }
  std::cout << "}\n";

  return 0;
}