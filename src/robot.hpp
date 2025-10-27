
#include <string>
#include <vector>

enum class HandType { NO_HAND = 0, PND_HAND = 1 };

class robot {
 public:
  virtual ~robot() = default;

  const std::vector<std::string>& fingerJointNames() const { return finger_joint_names_; }
  const std::vector<std::string>& linearActuatorNames() const { return linear_actuator_names_; }
  const HandType handType() const { return hand_type_; }

 protected:
  HandType hand_type_ = HandType::NO_HAND;
  std::vector<std::string> finger_joint_names_;
  std::vector<std::string> linear_actuator_names_;
};

class AdamLite : public robot {
 public:
  AdamLite() = default;
};

class AdamPro : public robot {
 public:
  AdamPro() {
    hand_type_ = HandType::PND_HAND;
    finger_joint_names_ = {"L_thumb_MCP_joint1", "L_thumb_MCP_joint2", "L_thumb_PIP_joint",  "L_thumb_DIP_joint",
                           "L_index_MCP_joint",  "L_index_DIP_joint",  "L_middle_MCP_joint", "L_middle_DIP_joint",
                           "L_ring_MCP_joint",   "L_ring_DIP_joint",   "L_pinky_MCP_joint",  "L_pinky_DIP_joint",
                           "R_thumb_MCP_joint1", "R_thumb_MCP_joint2", "R_thumb_PIP_joint",  "R_thumb_DIP_joint",
                           "R_index_MCP_joint",  "R_index_DIP_joint",  "R_middle_MCP_joint", "R_middle_DIP_joint",
                           "R_ring_MCP_joint",   "R_ring_DIP_joint",   "R_pinky_MCP_joint",  "R_pinky_DIP_joint"};
    linear_actuator_names_ = {"l_pinky", "l_ring", "l_middle", "l_index", "l_thumb", "l_thumb_lateral",
                              "r_pinky", "r_ring", "r_middle", "r_index", "r_thumb", "r_thumb_lateral"};
  }
};
