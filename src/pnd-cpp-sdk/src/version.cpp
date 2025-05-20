#include "version.hpp"

#include <cstdio>

namespace Pnd {

VersionNumber getCVersion() {
  int32_t maj, min, rev;
  pndGetLibraryVersion(&maj, &min, &rev);
  return VersionNumber(maj, min, rev);
}

// pndc++ VERSION
VersionNumber getCppVersion() { return VersionNumber(1, 5, 1); }

}  // namespace Pnd
