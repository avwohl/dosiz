//
// dosbox_conf.h — translate a dosemu::Config into a dosbox-staging .conf file
// on disk. Returns the temp file path (caller owns and should unlink).
//

#ifndef DOSEMU_DOSBOX_CONF_H
#define DOSEMU_DOSBOX_CONF_H

#include <string>
#include "config.h"

namespace dosemu {

// Writes a dosbox.conf into the given path (or a new temp file if path is
// empty). Returns the actual path written, or an empty string on error.
std::string write_dosbox_conf(const Config &cfg, const std::string &path = "");

// Locate the dosbox-staging binary. Order of precedence:
//   1. cfg.dosbox_binary (if set)
//   2. DOSEMU_DOSBOX env var
//   3. ../dosbox-staging/build/dosbox relative to the dosemu executable
//   4. "dosbox" on PATH
// Returns empty string if nothing is found.
std::string locate_dosbox_binary(const Config &cfg, const char *argv0);

} // namespace dosemu

#endif // DOSEMU_DOSBOX_CONF_H
