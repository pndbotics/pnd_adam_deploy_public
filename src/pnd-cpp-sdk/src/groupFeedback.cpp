#include "groupFeedback.hpp"

namespace Pnd {

GroupFeedback::GroupFeedback(size_t number_of_modules)
    : internal_(pndGroupFeedbackCreate(number_of_modules)),
      manage_pointer_lifetime_(true),
      number_of_modules_(number_of_modules) {
  for (size_t i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(pndGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::GroupFeedback(PndGroupFeedbackPtr group_feedback)
    : internal_(group_feedback),
      manage_pointer_lifetime_(false),
      number_of_modules_(pndGroupFeedbackGetSize(group_feedback)) {
  for (size_t i = 0; i < number_of_modules_; i++)
    feedbacks_.emplace_back(pndGroupFeedbackGetModuleFeedback(internal_, i));
}

GroupFeedback::~GroupFeedback() noexcept {
  if (manage_pointer_lifetime_ && internal_ != nullptr) pndGroupFeedbackRelease(internal_);
}

size_t GroupFeedback::size() const { return number_of_modules_; }

const PndFeedbackPtr &GroupFeedback::operator[](size_t index) const { return feedbacks_[index]; }

}  // namespace Pnd
