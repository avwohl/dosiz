/*
 * debug_settings.cpp -- Constructor reads all DOSIZ_* env vars once at
 * static init.  C++ guarantees namespace-scope objects with static
 * storage duration are constructed before main() runs, and the process
 * environment is fully populated before any constructor fires, so
 * reading getenv() in the ctor is safe.  No other global constructor
 * references g_debug, so initialization order across translation units
 * doesn't matter.
 */
#include "debug_settings.hpp"

#include <cstdlib>

namespace dosiz {

namespace {

// Present and non-empty (any value including "0" counts as set).
// Most call sites historically used `getenv("X") != nullptr`, which
// also treats empty string as set.  Preserve that by only requiring
// non-null.
inline bool env_present(const char* name) {
  return std::getenv(name) != nullptr;
}

// Returns raw pointer or nullptr if unset/empty.  Pointer lifetime
// matches the process environment -- safe to cache.
inline const char* env_str(const char* name) {
  const char* v = std::getenv(name);
  return (v != nullptr && *v != '\0') ? v : nullptr;
}

}  // namespace

void DebugSettings::reload() {
  *this = DebugSettings();
}

DebugSettings::DebugSettings() {
  // --- Runtime behavior switches ---
  dpmi_ring3    = env_present("DOSIZ_DPMI_RING3");
  dpmi_ring0    = env_present("DOSIZ_DPMI_RING0");
  force_dpmi    = env_present("DOSIZ_FORCE_DPMI");
  no_dpmi       = env_present("DOSIZ_NO_DPMI");
  le_as_mz      = env_present("DOSIZ_LE_AS_MZ");

  // --- Trace flags ---
  trace         = env_present("DOSIZ_TRACE");
  dpmi_trace    = env_present("DOSIZ_DPMI_TRACE");
  exc_trace     = env_present("DOSIZ_EXC_TRACE");
  ldt_trace     = env_present("DOSIZ_LDT_TRACE");
  simrm_trace   = env_present("DOSIZ_SIMRM_TRACE");
  stackwatch    = env_present("DOSIZ_STACKWATCH");
  int4b_trace   = env_present("DOSIZ_4B_TRACE");
  int4c_trace   = env_present("DOSIZ_4C_TRACE");
  open_trace    = env_present("DOSIZ_OPEN_TRACE");
  write_trace   = env_present("DOSIZ_WRITE_TRACE");
  cpu_trace     = env_present("DOSIZ_CPU_TRACE");

  // --- Strings ---
  path          = env_str("DOSIZ_PATH");
}

DebugSettings g_debug;  // static-storage-duration; constructor runs before main().

}  // namespace dosiz
