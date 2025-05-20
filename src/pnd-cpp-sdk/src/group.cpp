#include "group.hpp"

#include "groupCommand.hpp"
#include "groupFeedback.hpp"

namespace Pnd {

void callbackWrapper(PndGroupFeedbackPtr group_feedback, void *user_data) {
  reinterpret_cast<Group *>(user_data)->callAttachedHandlers(group_feedback);
}

void Group::callAttachedHandlers(PndGroupFeedbackPtr group_feedback) {
  // Wrap this:
  GroupFeedback wrapped_fbk(group_feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++) {
    GroupFeedbackHandler handler = handlers_[i];
    try {
      handler(wrapped_fbk);
    } catch (...) {
    }
  }
}

Group::Group(PndGroupPtr group, float initial_feedback_frequency, int32_t initial_command_lifetime)
    : internal_(group), number_of_modules_(pndGroupGetSize(internal_)) {
  if (initial_feedback_frequency != 0) pndGroupSetFeedbackFrequencyHz(internal_, initial_feedback_frequency);
  if (initial_command_lifetime != 0) pndGroupSetCommandLifetime(internal_, initial_command_lifetime);
}

Group::~Group() noexcept {
  if (internal_ != nullptr) {
    pndGroupRelease(internal_);
  }
}

int Group::size() { return number_of_modules_; }

bool Group::setCommandLifetimeMs(int32_t ms) { return (pndGroupSetCommandLifetime(internal_, ms) == PndStatusSuccess); }

bool Group::sendCommand(const GroupCommand &group_command) {
  return (pndGroupSendCommand(internal_, group_command.internal_) == PndStatusSuccess);
}

bool Group::sendFeedbackRequest(PndFeedbackCode feedbackCode) {
  return (pndGroupSendFeedbackRequest(internal_, feedbackCode) == PndStatusSuccess);
}

bool Group::getNextFeedback(GroupFeedback &feedback, int32_t timeout_ms) {
  return (pndGroupGetNextFeedback(internal_, feedback.internal_, timeout_ms) == PndStatusSuccess);
}

bool Group::setFeedbackFrequencyHz(float frequency) {
  return (pndGroupSetFeedbackFrequencyHz(internal_, frequency) == PndStatusSuccess);
}

float Group::getFeedbackFrequencyHz() { return pndGroupGetFeedbackFrequencyHz(internal_); }

void Group::addFeedbackHandler(GroupFeedbackHandler handler) {
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler);
  if (handlers_.size() == 1)  // (i.e., this was the first one)
    pndGroupRegisterFeedbackHandler(internal_, callbackWrapper, reinterpret_cast<void *>(this));
}

void Group::clearFeedbackHandlers() {
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  pndGroupClearFeedbackHandlers(internal_);
  handlers_.clear();
}

PndFeedbackErrorPtr Group::getError(int idx) { return pndGroupFeedbackError(internal_, idx); }
}  // namespace Pnd
