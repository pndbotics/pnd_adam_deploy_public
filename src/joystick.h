#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <thread>

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

// #define XBOX_BUTTON_A       0x00
// #define XBOX_BUTTON_B       0x01
// #define XBOX_BUTTON_X       0x02
// #define XBOX_BUTTON_Y       0x03
// #define XBOX_BUTTON_LB      0x04
// #define XBOX_BUTTON_RB      0x05
// #define XBOX_BUTTON_START   0x06
// #define XBOX_BUTTON_BACK    0x07
// #define XBOX_BUTTON_HOME    0x08
// #define XBOX_BUTTON_LO      0x09    /* 左摇杆按键 */
// #define XBOX_BUTTON_RO      0x0a    /* 右摇杆按键 */
//
// #define XBOX_BUTTON_ON      0x01
// #define XBOX_BUTTON_OFF     0x00
//
// #define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
// #define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
// #define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */
// #define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */
// #define XBOX_AXIS_LT        0x02
// #define XBOX_AXIS_RT        0x05
// #define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
// #define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */

// bluetooth map
#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x03
#define XBOX_BUTTON_Y 0x04
#define XBOX_BUTTON_LB 0x06
#define XBOX_BUTTON_RB 0x07
#define XBOX_BUTTON_START 0x02
#define XBOX_BUTTON_BACK 0x05
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x02 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x03 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x05
#define XBOX_AXIS_RT 0x04
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

typedef struct xbox_map {
  int time;
  int a;
  int b;
  int x;
  int y;
  int lb;
  int rb;
  int start;
  int back;
  int home;
  int lo;
  int ro;

  int lx;
  int ly;
  int rx;
  int ry;
  int lt;
  int rt;
  int xx;
  int yy;

} xbox_map_t;

enum class GaitEnum { Stand, Walk, Run };

class JsHum {
 private:
  JsHum();

 public:
  static JsHum &getInst() {
    static JsHum instance;
    return instance;
  }
  ~JsHum();
  void init();
  void xbox_run();
  /**
   * @brief Get the state change object
   * A: gotoZero
   * B: gotoStop
   * X: gotoMLP
   * Y:
   * @return std::string
   */
  std::string getStateChange();
  GaitEnum getCommandGait();
  void changeCommandGait(GaitEnum cg);
  GaitEnum command_gait = GaitEnum::Stand;
  int dead_area = 5000;
  int max_value = 32767;
  double lx_dir = -1.0;
  double ly_dir = -1.0;

  double rx_dir = -1.0;
  double ry_dir = -1.0;

  double xx_dir = -1.0;
  double yy_dir = -1.0;
  // get command  velocity for walk
  double getWalkXDirectionSpeed() const;
  double max_speed_x = 0.25;  // - x
  double min_speed_x = -0.5;  // +x

  double getWalkYDirectionSpeed() const;
  double max_speed_y = 0.25;
  double min_speed_y = -0.25;

  double getWalkYawDirectionSpeed() const;
  double max_speed_yaw = 0.4;
  double min_speed_yaw = -0.4;

  double getWalkXDirectionSpeedOffset();
  double delta_offset_x = 0.05;
  double last_value_x = 0;

  double getWalkYDirectionSpeedOffset();
  double delta_offset_y = 0.05;
  double last_value_y = 0;

  // get command position for stand
  double getStandXDirectionPosition() const;
  double max_position_x = 0.1;
  double min_position_x = -0.1;

  double getStandYDirectionPosition() const;
  double max_position_y = 0.06;
  double min_position_y = -0.06;

  double getStandZDirectionPosition() const;
  double max_position_z = 0.2;
  double min_position_z = -0.05;

  double getStandRollDirectionPosition();
  double max_position_roll = 1.0;
  double min_position_roll = -1.0;
  double roll_delta_cmd = 0.01;
  double roll_cmd = 0.0;

  double getStandPitchDirectionPosition();
  double max_position_pitch = 0.5;
  double min_position_pitch = -0.5;
  double pitch_delta_cmd = 0.01;
  double pitch_cmd = 0.0;

  int getMotionState();
  bool disableJoints();

 private:
  static int xboxOpen(const char *file_name);
  int xboxMapRead(xbox_map_t *map) const;
  void xboxClose() const;
  int xboxInit();
  int xboxRead(xbox_map_t *xbox_m);

 private:
  // fsmstate
  std::string current_fsmstate_command_;
  //
  xbox_map_t xbox_m_{};
  // pthread_t xbox_thread_;
  std::thread xbox_thread_;
  std::atomic<bool> exit_flag_;
  // xbox
  int xbox_fd_{};
};
#endif
