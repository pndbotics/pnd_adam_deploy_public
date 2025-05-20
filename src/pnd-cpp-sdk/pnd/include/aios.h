#pragma once
#include <stddef.h>
#include <stdint.h>

/* Use C linkage when compiling this C library header from C++ */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * API call status codes
 */
typedef enum PndStatusCode {
  PndStatusFailure = -1,  // Failure
  PndStatusSuccess = 0,   // Success
} PndStatusCode;

/**
 * motor number
 */
typedef enum MotorNumber {
  MOTOR_NUMBER_0 = 0,  // reserved
  MOTOR_NUMBER_1 = 1,  // in use
} MotorNumber;

/**
 * Initialize the actuator state
 */
typedef enum AxisState {
  AXIS_STATE_UNKNOW = -1,
  AXIS_STATE_IDLE = 1,    // idle state
  AXIS_STATE_ENABLE = 8,  // enabling state
} AxisState;

/**
 * Actuator control mode
 */
typedef enum ControlMode {
  VOLTAGE_CONTROL = 0,     //
  CURRENT_CONTROL = 1,     //
  VELOCITY_CONTROL = 2,    //
  POSITION_CONTROL = 3,    //
  TRAJECTORY_CONTROL = 4,  //
} ControlMode;

/**
 * Actuator input mode
 */
typedef enum InputMode {
  INPUT_MODE_INACTIVE = 0,
  INPUT_MODE_PASSTHROUGH = 1,
  INPUT_MODE_VEL_RAMP = 2,
  INPUT_MODE_POS_FILTER = 3,
  INPUT_MODE_TRAP_TRAJ = 5,
  INPUT_MODE_TORQUE_RAMP = 6,
} InputMode;

////////////////////////////////////////////////////////////////////////////////
/// Command Enums
////////////////////////////////////////////////////////////////////////////////
typedef enum PndCommandEnum {
  UNKNOW = -1,
  // float
  PndCommandCurrent = CURRENT_CONTROL,
  PndCommandVelocity = VELOCITY_CONTROL,
  PndCommandPosition = POSITION_CONTROL,
  PndCommandVelocityRamp,
  PndCommandTrapezoidal,
  PndCommandTorquePt,    // fw2.0
  PndCommandVelocityPt,  // fw2.0
  PndCommandPositionPt,  // fw2.0
  PndCommandEnable,

  // bool
  PndCommandReboot,
  PndCommandGetError,
  PndCommandClearError,
  PndCommandGetMotorRotorAbsPos,

  PndCommandResetLinearCount,        // Reset Linear Count
  PndCommandMotionControllerConfig,  // Motion Controller Config
  PndCommandMotorConfig,             // Motor Config
  PndCommandTrapTraj,                // TRAP-TRAJ
  PndCommandSaveConfig,              // Save Config

  PndCommandNetworkSetting,  // network setting

  PndCommandLatencyTest,  // For Latency Test

  // Rcu
  PndCommandRcuPower,
  PndCommandRcuAV5V,
  PndCommandRcuV5V,
  PndCommandRcuV5VA,
  PndCommandRcuV5VB,
  PndCommandRcu12V,
  PndCommandRcuFan12V,
  PndCommandRcu19V,
  PndCommandRcuBrake,
  PndCommandRcuAdcOffset,
  PndCommandRcuFirmwareUpgrade,

  // Encoder
  PndEncoderCommandGetAngle
} PndCommandEnum;

////////////////////////////////////////////////////////////////////////////////
/// Feedback Enums
////////////////////////////////////////////////////////////////////////////////
typedef enum PndFeedbackCode {
  PndFeedbackAll = 1,  //
  PndFeedbackCVP = 2,  // Used for ovtaining actuator position,velocity and
                       // current at high speed
} PndFeedbackCode;

////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////
/**
 * To return group objects, only one Lookup object is required per application
 */
typedef struct PndLookup_ *PndLookupPtr;

/**
 * The entry generated after the lookup
 */
typedef struct PndLookupEntryList_ *PndLookupEntryListPtr;

/**
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 */
typedef struct PndGroup_ *PndGroupPtr;

/**
 * Encapsulates feedback received from a module
 */
typedef struct PndGroupFeedback_ *PndGroupFeedbackPtr;

/**
 * Encapsulates data to be sent to a module
 */
typedef struct PndGroupCommand_ *PndGroupCommandPtr;

typedef void (*GroupFeedbackHandlerFunction)(PndGroupFeedbackPtr fbk, void *user_data);

////////////////////////////////////////////////////////////////////////////////
/// Structures
////////////////////////////////////////////////////////////////////////////////
typedef struct FeedbackError_ {
  char axis[16];
  char motor[16];
  char encoder[16];
} FeedbackError;

typedef struct FeedbackError_ *PndFeedbackErrorPtr;

typedef struct MotionControllerConfig_ {
  float pos_gain;
  float vel_gain;
  float vel_integrator_gain;
  float vel_limit;
  float vel_limit_tolerance;
} MotionControllerConfig;

typedef struct MotorConfig_ {
  int current_lim;
  int current_lim_margin;
  int inverter_temp_limit_lower;
  int inverter_temp_limit_upper;
  int requested_current_range;
  int current_control_bandwidth;
} MotorConfig;

typedef struct TrapTraj_ {
  int accel_limit;
  int decel_limit;
  int vel_limit;
} TrapTraj;

typedef struct NetworkSetting_ {
  bool dhcp_enable;
  char SSID[16];
  char password[32];
  char name[32];
  char staticIP[20];
  char gateway[20];
  char subnet[20];
  char dns_1[20];
  char dns_2[20];
} NetworkSetting;

typedef struct Feedback_ {
  float position;  // Current motor position. if position=1, the motor angle is 360 degree. if position=-1, the motor
                   // angle is -360 degree.
  float velocity;  // Current motor velocity. unit:turn/s
  float current;   // Current motor current unit: A
  int error_code;  // Error code. detail in pnd wiki
  float voltage;   // Current motor voltage. unit: V
  float motor_temp_m0;  // reserved
  float motor_temp_m1;
  float inverter_temp_m0;  // reserved
  float inverter_temp_m1;
  bool drive_status;
  bool enabled;            // Current motor enabled status
  char ip[16];             // ip address
  char serial_number[13];  // serial number
  char connect_mode[9];    // motor connect mode.("ethernet" or "wifi")
  char model[12];          // device type
  char mac_address[18];    // mac address
  char hw_version[6];      // hardware version
  char fw_version[6];      // firmware version

  MotionControllerConfig motion_ctrl_config;
  MotorConfig motor_config;
  TrapTraj trap_traj;

  NetworkSetting network_setting;
  float encoder_angle;  // Current encoder angle unit: degree
  float motor_rotor_abs_pos; // Current motor rotor absolute position; range: 0~16384
} Feedback;

typedef struct Feedback_ *PndFeedbackPtr;

typedef struct PosPtInfo_ {
  float pos;  // motor position; if pos=1.0, the motor rotate one revolution clockwise (360 degree). if pos=-1.0, the
              // motor rotate one revolution counterclockwise (-360 degree)
  float vel_ff;     // velocity feedforward
  float torque_ff;  // torque feedforward
} PosPtInfo;

typedef struct RcuBrake_ {
  bool brake_enable;
  float brake_overvoltage;
  int brake_factor;
} RcuBrake;

typedef struct Command_ {
  float float_fields_;
  bool bool_fields_;

  PosPtInfo pos_pt_info_;

  MotionControllerConfig *motion_ctrl_config;
  MotorConfig *motor_config;
  TrapTraj *trap_traj;

  NetworkSetting *network_setting;

  // Rcu
  RcuBrake rcu_brake_;

  PndCommandEnum commandEnum;
} Command;

typedef struct Command_ *PndCommandPtr;

////////////////////////////////////////////////////////////////////////////////
/// Lookup API
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Create a Lookup instance.
 * Lookup created by this function must be released with 'pndLookupRelease'
 * when no longer needed.
 *
 * Note that this call invokes a background thread to query the network for
 * modules at regular intervals.
 *
 * @param addr A string pointer to a broadcast address.
 *             eg: "192.168.100.255,192.168.101.255"
 *
 * @param addr_len the length of the buffer `addr`. This must be zero
 *                 if `addr` is null.
 */
PndLookupPtr pndLookupCreate(const char *addr, size_t addr_len);

/**
 * @brief Frees resources created by the lookup object.
 *
 * Lookup object should no longer be used after this function is called!
 * Note that background query thread is stopped by this function.
 *
 * @param lookup Object created by `pndLookupCreate`.
 */
void pndLookupRelease(PndLookupPtr lookup);

/**
 * @brief Sets the broadcast address for the lookup.
 *
 * @param lookup Object created by `pndLookupCreate`.
 * @param addr A string pointer to a broadcast address.
 *             eg: "192.168.100.255,192.168.101.255"
 */
void pndLookupSetNetworks(PndLookupPtr lookup, const char *addr);

void pndLookupGetCtrlBoxIP(PndLookupPtr lookup, char *res);

/**
 * @brief sets the lookup request rate [Hz]
 *
 * @param lookup Object created by `pndLookupCreate`.
 * @param frequency The recommended range is 0 to 100
 *
 * @returns PndStatusSuccess on success, PndStatusInvalidArgument on
 *          negative or non-finite frequency
 */
PndStatusCode pndLookupSetLookupFrequencyHz(PndLookupPtr lookup, const float frequency);

/**
 * @brief gets the lookup request rate [Hz]
 */
const float pndLookupGetLookupFrequencyHz(PndLookupPtr lookup);

/**
 * @brief Write the found object to PndLookupEntryListPtr and return it.
 *
 * @param lookup Object created by `pndLookupCreate`.
 */
PndLookupEntryListPtr pndCreateLookupEntryList(PndLookupPtr lookup);

/**
 * @brief Gets the number of entries in the lookup entry list.
 *
 * @param lookup_list Object created by `pndCreateLookupEntryList`.
 */
size_t pndLookupEntryListGetSize(PndLookupEntryListPtr lookup_list);

typedef struct DeviceInfo_ {
  char ip[16];
  char serial_number[13];
  char model[20];
  char mac_address[18];
} DeviceInfo;
typedef struct DeviceList_ {
  size_t size;
  DeviceInfo *deviceInfo;
} DeviceList;
DeviceList *pndLookupGetDeviceList(PndLookupPtr lookup);
PndStatusCode pndLookupDeviceListFree(DeviceList *device_list);

/**
 * @brief Gets the name of the given entry in the lookup entry list. Must be a
 *        valid index.
 *
 * @param lookup_list Object created by `pndCreateLookupEntryList`.
 * @param index The entry index that is being queried.
 * @param name An allocated buffer of length 'length'
 * @param length the length of the provided buffer. After calling this function,
 *               the value dereferenced will be updated with the length of the
 *               string plus the null character. This argument must not be NULL.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndLookupEntryListGetName(PndLookupEntryListPtr lookup_list, size_t index, char *name, size_t *length);

/**
 * @brief Gets the family of the given entry in the lookup entry list. Must be a
 *        valid index.
 *
 * @param lookup_list Object created by `pndCreateLookupEntryList`.
 * @param index The entry index that is being queried.
 * @param family An allocated buffer of length 'length'
 * @param length the length of the provided buffer. After calling this function,
 *               the value dereferenced will be updated with the length of the
 *               string plus the null character. This argument must not be NULL.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndLookupEntryListGetFamily(PndLookupEntryListPtr lookup_list, size_t index, char *family,
                                          size_t *length);

/**
 * @brief Gets the serialNumber of the given entry in the lookup entry list.
 * Must be a valid index.
 *
 * @param lookup_list Object created by `pndCreateLookupEntryList`.
 * @param index The entry index that is being queried.
 * @param serialNumber An allocated buffer of length 'length'
 * @param length the length of the provided buffer. After calling this function,
 *               the value dereferenced will be updated with the length of the
 *               string plus the null character. This argument must not be NULL.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndLookupEntryListGetSerialNumber(PndLookupEntryListPtr lookup_list, size_t index, char *serialNumber,
                                                size_t *length);

/**
 * @brief Release resources for a given lookup entry list; list should not be
 * used after this call.
 * @param lookup_list Object created by `pndCreateLookupEntryList`.
 */
void pndLookupEntryListRelease(PndLookupEntryListPtr lookup_list);

////////////////////////////////////////////////////////////////////////////////
/// Feedback API
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Creates a GroupFeedback for a group with the specified number of
 * modules.
 *
 * @param size The number of modules in the group.
 *
 * @returns A pointer to a new GroupFeedback object. This must be released
 * with \c pndGroupFeedbackRelease(PndGroupFeedbackPtr).
 */
PndGroupFeedbackPtr pndGroupFeedbackCreate(size_t size);

/**
 * @brief Return the number of modules in this group Feedback.
 *
 * @returns The number of module feedbacks in this group feedback.
 */
size_t pndGroupFeedbackGetSize(PndGroupFeedbackPtr feedback);

/**
 * @brief Get an individual feedback for a particular module at index
 * \c module_index.
 *
 * @param module_index The index to retrieve the module feedback; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * @returns The feedback corresponding to the module at index \c module_index.
 */
PndFeedbackPtr pndGroupFeedbackGetModuleFeedback(PndGroupFeedbackPtr feedback, size_t module_index);

/**
 * @brief Get the duration of the command sent to recv.
 *
 * @returns The duration. unit: us.
 */
const int pndGroupFeedbackGetDuration(PndGroupFeedbackPtr feedback);

/**
 * @brief Frees resources created by the GroupFeedback object.
 *
 * The GroupFeedbackPtr must not be used after this function is called.
 */
void pndGroupFeedbackRelease(PndGroupFeedbackPtr feedback);

////////////////////////////////////////////////////////////////////////////////
/// Group API
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Create a group with all modules known to the lookup with the given
 * family.
 *
 * Group contains all modules with the given family, regardless of name.
 *
 * @param lookup Object created by `pndLookupCreate`.
 * @param family The given family of the modules, as viewable in the
 * PndStudio Must be a null-terminated string, and must not be NULL.
 * @param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 *
 * @returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
PndGroupPtr pndGroupCreateFromFamily(PndLookupPtr lookup, const char *family, int32_t timeout_ms);

PndGroupPtr pndGroupCreateFromIps(PndLookupPtr lookup, const char *ips, const size_t ips_len);

/**
 * @brief Returns the number of modules in a group.
 *
 * @param group The group to send this command to.
 *
 * @returns the number of modules in the group.
 */
size_t pndGroupGetSize(PndGroupPtr group);

/**
 * @brief Sends a command to the given group without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * @param group The group to send this command to.
 * @param command The PndGroupCommand object containing information to be
 * sent to the group.
 *
 * @returns PndStatusSuccess if the command was successfully sent, otherwise
 * a failure code.
 */
PndStatusCode pndGroupSendCommand(PndGroupPtr group, PndGroupCommandPtr command);

/**
 * @brief Returns the current command lifetime, in milliseconds.
 *
 * @param group Which group is being queried.
 *
 * @returns The current command lifetime, in milliseconds. A value of '0'
 * indicates that commands remain active until the next command is received.
 */
PndStatusCode pndGroupSetCommandLifetime(PndGroupPtr group, int32_t lifetime_ms);

/**
 * @brief Sets the feedback request loop frequency (in Hz).
 *
 * // TODO:
 * The group is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * @param group Which group this frequency set is for.
 * @param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndGroupSetFeedbackFrequencyHz(PndGroupPtr group, const float frequency);

/**
 * @brief Returns the current feedback request loop frequency (in Hz).
 *
 * @param group Which group is being queried.
 *
 * @returns The current feedback request loop frequency (in Hz).
 */
float pndGroupGetFeedbackFrequencyHz(PndGroupPtr group);

/**
 * @brief Requests feedback from the group.
 *
 * Sends a background request to the modules in the group; if/when all modules
 * return feedback, any associated handler functions are called. This returned
 * feedback is also stored to be returned by the next call to
 * pndGroupGetNextFeedback (any previously returned data is discarded).
 *
 * @param group The group to return feedback from.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndGroupSendFeedbackRequest(PndGroupPtr group, PndFeedbackCode feedbackCode);

/**
 * @brief Returns the most recently stored feedback from a sent feedback
 * request, or returns the next one received (up to the requested timeout).
 *
 * Note that a feedback request can be sent either with the
 * pndGroupSendFeedbackRequest function, or by setting a background feedback
 * frequency with pndGroupSetFeedbackFrequencyHz.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * @param group The group to return feedback from.
 * @param feedback On success, the feedback read from the group are written
 * into this structure.
 * @param timeout_ms Indicates how many milliseconds to wait for feedback.
 * For typical networks, '15' ms is a value that can be reasonably expected to
 * allow for a round trip transmission after the last 'send feedback request'
 * call.
 *
 * @returns PndStatusSuccess on success，PndStatusFailure on failed.
 */
PndStatusCode pndGroupGetNextFeedback(PndGroupPtr group, PndGroupFeedbackPtr feedback, int32_t timeout_ms);

// TODO:
PndStatusCode pndGroupRegisterFeedbackHandler(PndGroupPtr group, GroupFeedbackHandlerFunction handler, void *user_data);

// TODO:
void pndGroupClearFeedbackHandlers(PndGroupPtr group);

/**
 * @brief Release resources for a given group; group should not be used after
 * this call.
 *
 * @param group A valid PndGroup object.
 */
void pndGroupRelease(PndGroupPtr group);

/**
 * @brief Creates a GroupCommand for a group with the specified number of
 * modules.
 *
 * @param size The number of modules in the group.
 *
 * @returns A pointer to a new GroupCommand object. This must be released
 * with \c pndGroupCommandRelease(PndGroupCommandPtr).
 */
PndGroupCommandPtr pndGroupCommandCreate(size_t size);

/**
 * @brief Return the number of modules in this group Command.
 *
 * @returns The number of module commands in this group command.
 */
size_t pndGroupCommandGetSize(PndGroupCommandPtr command);

/**
 * @brief Get an individual command for a particular module at index
 * \c module_index.
 *
 * @param module_index The index to retrieve the module command; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * @returns The command corresponding to the module at index \c module_index.
 */
PndCommandPtr pndGroupCommandGetModuleCommand(PndGroupCommandPtr command, size_t module_index);

/**
 * @brief Sets the type of a set of commands
 *
 * @param command_enum Any value in PndCommandEnum
 *
 * @returns The command corresponding to the module at index \c module_index.
 */
bool pndGroupCommandSetType(PndGroupCommandPtr command, PndCommandEnum command_enum);
/**
 * @brief Clears all data in the GroupCommand object.
 */
void pndGroupCommandClear(PndGroupCommandPtr command);

/**
 * @brief Frees resources created by the GroupCommand object.
 *
 * The GroupCommandPtr must not be used after this function is called.
 */
void pndGroupCommandRelease(PndGroupCommandPtr command);

/**
 * @brief Gets the actuator error message
 */
PndFeedbackErrorPtr pndGroupFeedbackError(PndGroupPtr group, int idx);

/**
 * @brief Set the log print level.
 *
 * @param mode "DEBUG" "INFO" "WARN" "ERROR"
 * @return PndStatusCode
 */
PndStatusCode pndSetLogLevel(const char *console_mode, const char *file_mode);

/**
 * \brief Get the version of the library
 *
 * All parameters must not be NULL.
 *
 * \return PndStatusSuccess on success, otherwise
 * PndStatusInvalidArgument if a parameter is NULL.
 */
PndStatusCode pndGetLibraryVersion(int32_t *major, int32_t *minor, int32_t *revision);

#ifdef __cplusplus
}  // extern "C"
#endif
