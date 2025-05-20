#pragma once
#include "version.hpp"

namespace Pnd {

/**
 * @brief Returns the version numbers for the Pnd Adam deploy sdk API.
 */
VersionNumber getAdamDeployVersion() { return VersionNumber(1, 4, 0); };

}  // namespace Pnd
