#include "joystick.h"

#include <iostream>

JsHum::JsHum() { exit_flag_.store(false); }

JsHum::~JsHum() {
  exit_flag_.store(true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "joystick end" << std::endl;
}

int JsHum::xboxOpen(const char *file_name) {
  int xbox_fd;

  xbox_fd = open(file_name, O_RDONLY);
  if (xbox_fd < 0) {
    perror("open");
    return -1;
  }

  return xbox_fd;
}

int JsHum::xboxMapRead(xbox_map_t *map) const {
  int len, type, number, value;
  struct js_event js;

  len = read(xbox_fd_, &js, sizeof(struct js_event));
  if (len < 0) {
    perror("read");
    return -1;
  }

  type = js.type;
  number = js.number;
  value = js.value;

  map->time = js.time;

  if (type == JS_EVENT_BUTTON) {
    switch (number) {
      case XBOX_BUTTON_A:
        map->a = value;
        break;

      case XBOX_BUTTON_B:
        map->b = value;
        break;

      case XBOX_BUTTON_X:
        map->x = value;
        break;

      case XBOX_BUTTON_Y:
        map->y = value;
        break;

      case XBOX_BUTTON_LB:
        map->lb = value;
        break;

      case XBOX_BUTTON_RB:
        map->rb = value;
        break;

      case XBOX_BUTTON_START:
        map->start = value;
        break;

      case XBOX_BUTTON_BACK:
        map->back = value;
        break;

      case XBOX_BUTTON_HOME:
        map->home = value;
        break;

      case XBOX_BUTTON_LO:
        map->lo = value;
        break;

      case XBOX_BUTTON_RO:
        map->ro = value;
        break;

      default:
        break;
    }
  } else if (type == JS_EVENT_AXIS) {
    switch (number) {
      case XBOX_AXIS_LX:
        map->lx = value;
        break;

      case XBOX_AXIS_LY:
        map->ly = value;
        break;

      case XBOX_AXIS_RX:
        map->rx = value;
        break;

      case XBOX_AXIS_RY:
        map->ry = value;
        break;

      case XBOX_AXIS_LT:
        map->lt = value;
        break;

      case XBOX_AXIS_RT:
        map->rt = value;
        break;

      case XBOX_AXIS_XX:
        map->xx = value;
        break;

      case XBOX_AXIS_YY:
        map->yy = value;
        break;

      default:
        break;
    }
  } else {
    /* Init do nothing */
  }

  return len;
}

void JsHum::xboxClose() const { close(xbox_fd_); }

int JsHum::xboxInit() {
  xbox_fd_ = xboxOpen("/dev/input/js0");
  if (xbox_fd_ < 0) {
    return -1;
  }
  return 0;
}

int JsHum::xboxRead(xbox_map_t *xbox_m) {
  int len = xboxMapRead(xbox_m);
  if (len < 0) {
    return -1;
  }
  return 0;
}

void JsHum::init() {
  // int ret = -1;
  xbox_m_.a = 0.0;
  xbox_m_.b = 0.0;
  xbox_m_.x = 0.0;
  xbox_m_.y = 0.0;
  xbox_m_.lx = 0.0;
  xbox_m_.ly = 0.0;
  xbox_m_.rx = 0.0;
  xbox_m_.ry = 0.0;
  xbox_m_.xx = 0.0;
  xbox_m_.yy = 0.0;
  xbox_m_.lb = 0.0;
  xbox_m_.rb = 0.0;
  xbox_m_.lt = -32767;
  xbox_m_.rt = -32767;
  current_fsmstate_command_ = "gotoStop";
  // ret = pthread_create(&Joystick::xbox_thread_, NULL, Joystick::xbox_run, NULL);
  xbox_thread_ = std::thread(&JsHum::xbox_run, this);
  xbox_thread_.detach();
}

void JsHum::xbox_run() {
  int len, ret;

  while (1) {
    ret = xboxInit();
    if (ret < 0) {
      printf("xbox init fail\n");
      usleep(1000 * 1000);
    } else {
      std::cout << "xbox connect!" << std::endl;
    }
    while (ret == 0) {
      len = xboxRead(&xbox_m_);
      if (len < 0) {
        std::cout << "xbox disconnect!" << std::endl;
        break;
      }

      // printf("A:%d B:%d X:%d Y:%d\n", xbox_m_.a, xbox_m_.b, xbox_m_.x, xbox_m_.y);
      // printf("ST:%d BK:%d HM:%d\n", xbox_m_.start, xbox_m_.back, xbox_m_.home);
      // printf("LX:%d LY:%d RX:%d RY:%d\n", xbox_m_.lx, xbox_m_.ly, xbox_m_.rx, xbox_m_.ry);
      // printf("LB:%d LT:%d RB:%d RT:%d\n", xbox_m_.lb, xbox_m_.lt, xbox_m_.rb, xbox_m_.rt);
      // printf("XX:%d YY:%d\n", xbox_m_.xx, xbox_m_.yy);
      //
      // fflush(stdout);

      usleep(10 * 1000);
    }
    if (exit_flag_.load()) {
      break;
    }
  }
  std::cout << "xbox run end" << std::endl;
}

std::string JsHum::getStateChange() {
  if (xbox_m_.a == 1.0) {
    current_fsmstate_command_ = "gotoZero";
  } else if (xbox_m_.b == 1.0) {
    current_fsmstate_command_ = "gotoStop";
  } else if (xbox_m_.x == 1.0) {
    current_fsmstate_command_ = "gotoMLP";
  }
  return current_fsmstate_command_;
}

GaitEnum JsHum::getCommandGait() {
  if (xbox_m_.x == 1.0) {
    command_gait = GaitEnum::Stand;
  } else if (xbox_m_.y == 1.0) {
    command_gait = GaitEnum::Walk;
  } else if (xbox_m_.rb == 1.0) {
    command_gait = GaitEnum::Run;
  }
  return command_gait;
}
void JsHum::changeCommandGait(GaitEnum cg) { command_gait = cg; }
double JsHum::getWalkXDirectionSpeed() const {
  int lt_value = xbox_m_.lt;
  if (lt_value < 1000) {
    int x_value = xbox_m_.ly;
    if ((abs(x_value) > dead_area) && (abs(x_value) <= max_value)) {
      if (x_value > 0) {
        return ly_dir * max_speed_x * ((double)(abs(x_value) - dead_area) / (max_value - dead_area));
      } else {
        return ly_dir * min_speed_x * ((double)(abs(x_value) - dead_area) / (max_value - dead_area));
      }
    } else {
      return 0.0;
    }
  } else {
    return 0.25;
  }
}

double JsHum::getWalkYDirectionSpeed() const {
  int y_value = xbox_m_.lx;
  if ((abs(y_value) > dead_area) && (abs(y_value) <= max_value)) {
    if (y_value > 0) {
      return lx_dir * max_speed_y * ((double)(abs(y_value) - dead_area) / (max_value - dead_area));
    } else {
      return lx_dir * min_speed_y * ((double)(abs(y_value) - dead_area) / (max_value - dead_area));
    }
  } else {
    return 0.0;
  }
}

double JsHum::getWalkYawDirectionSpeed() const {
  int yaw_value = xbox_m_.rx;
  if ((abs(yaw_value) > dead_area) && (abs(yaw_value) <= max_value)) {
    if (yaw_value > 0) {
      return rx_dir * max_speed_yaw * ((double)(abs(yaw_value) - dead_area) / (max_value - dead_area));
    } else {
      return rx_dir * min_speed_yaw * ((double)(abs(yaw_value) - dead_area) / (max_value - dead_area));
    }
  } else {
    return 0.0;
  }
}

double JsHum::getWalkXDirectionSpeedOffset() {
  int x_value = xbox_m_.yy;
  if ((x_value < -30000) && (abs(last_value_x) < 500)) {
    last_value_x = x_value;
    return delta_offset_x;
  } else if ((x_value > 30000) && (abs(last_value_x) < 500)) {
    last_value_x = x_value;
    return -delta_offset_x;
  } else {
    last_value_x = x_value;
    return 0.0;
  }
}

double JsHum::getWalkYDirectionSpeedOffset() {
  int y_value = xbox_m_.xx;
  if ((y_value < -30000) && (abs(last_value_y) < 500)) {
    last_value_y = y_value;
    return delta_offset_y;
  } else if ((y_value > 30000) && (abs(last_value_y) < 500)) {
    last_value_y = y_value;
    return -delta_offset_y;
  } else {
    last_value_y = y_value;
    return 0.0;
  }
}

// get command position for stand
double JsHum::getStandXDirectionPosition() const {
  int x_value = xbox_m_.ly;
  if ((abs(x_value) > dead_area) && (abs(x_value) <= max_value)) {
    if (x_value > 0) {
      return ly_dir * max_position_x * ((double)(abs(x_value) - dead_area) / (max_value - dead_area));
    } else {
      return ly_dir * min_position_x * ((double)(abs(x_value) - dead_area) / (max_value - dead_area));
    }
  } else {
    return 0.0;
  }
}

double JsHum::getStandYDirectionPosition() const {
  int y_value = xbox_m_.lx;
  if ((abs(y_value) > dead_area) && (abs(y_value) <= max_value)) {
    if (y_value > 0) {
      return lx_dir * max_position_y * ((double)(abs(y_value) - dead_area) / (max_value - dead_area));
    } else {
      return lx_dir * min_position_y * ((double)(abs(y_value) - dead_area) / (max_value - dead_area));
    }
  } else {
    return 0.0;
  }
}

double JsHum::getStandZDirectionPosition() const {
  int z_value = xbox_m_.ry;
  if ((abs(z_value) > dead_area) && (abs(z_value) <= max_value)) {
    if (z_value > 0) {
      return ry_dir * max_position_z * ((double)(abs(z_value) - dead_area) / (max_value - dead_area));
    } else {
      return ry_dir * min_position_z * ((double)(abs(z_value) - dead_area) / (max_value - dead_area));
    }
  } else {
    return 0.0;
  }
}

double JsHum::getStandRollDirectionPosition() {
  int x_value = xbox_m_.yy;
  if ((x_value < -30000)) {
    if (roll_cmd < max_position_roll) {
      roll_cmd += roll_delta_cmd;
    } else {
      roll_cmd = max_position_roll;
    }
    return roll_cmd;
  } else if ((x_value > 30000)) {
    if (roll_cmd > min_position_roll) {
      roll_cmd -= roll_delta_cmd;
    } else {
      roll_cmd = min_position_roll;
    }
    return roll_cmd;
  } else {
    return roll_cmd;
  }
}

double JsHum::getStandPitchDirectionPosition() {
  int x_value = xbox_m_.xx;
  if ((x_value < -30000)) {
    if (pitch_cmd < max_position_pitch) {
      pitch_cmd += pitch_delta_cmd;
    } else {
      pitch_cmd = max_position_pitch;
    }
    return pitch_cmd;
  } else if ((x_value > 30000)) {
    if (pitch_cmd > min_position_pitch) {
      pitch_cmd -= pitch_delta_cmd;
    } else {
      pitch_cmd = min_position_pitch;
    }
    return pitch_cmd;
  } else {
    return pitch_cmd;
  }
}

int JsHum::getMotionState() {
  int x_value = xbox_m_.xx;
  if (x_value < -30000) {
    return -1;
  } else if (x_value > 30000) {
    return 1;
  } else {
    return 0;
  }
}

bool JsHum::disableJoints() {
  if (xbox_m_.lt > 1000 && xbox_m_.rt > 1000) {
    return true;
  } else {
    return false;
  }
}