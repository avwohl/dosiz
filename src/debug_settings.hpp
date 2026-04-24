/*
 * debug_settings.hpp -- env-var snapshot for dosiz.
 *
 * Constructor reads every DOSIZ_* env var once at static-init (before
 * main() runs).  Call sites access flags through `g_debug.<field>`
 * instead of repeatedly calling getenv().  Modeled on DebugSettings
 * from the iospharo VM.
 */
#pragma once

namespace dosiz {

struct DebugSettings {
  DebugSettings();
  void reload();

  // Runtime behavior switches
  bool dpmi_ring3;       // DOSIZ_DPMI_RING3 (opt-in, now default on)
  bool dpmi_ring0;       // DOSIZ_DPMI_RING0 (opt-out for legacy ring-0 fixtures)
  bool force_dpmi;       // DOSIZ_FORCE_DPMI
  bool no_dpmi;          // DOSIZ_NO_DPMI
  bool le_as_mz;         // DOSIZ_LE_AS_MZ

  // Trace flags
  bool trace;            // DOSIZ_TRACE
  bool dpmi_trace;       // DOSIZ_DPMI_TRACE
  bool exc_trace;        // DOSIZ_EXC_TRACE
  bool ldt_trace;        // DOSIZ_LDT_TRACE
  bool simrm_trace;      // DOSIZ_SIMRM_TRACE
  bool stackwatch;       // DOSIZ_STACKWATCH
  bool int4b_trace;      // DOSIZ_4B_TRACE
  bool int4c_trace;      // DOSIZ_4C_TRACE
  bool open_trace;       // DOSIZ_OPEN_TRACE
  bool write_trace;      // DOSIZ_WRITE_TRACE
  bool cpu_trace;        // DOSIZ_CPU_TRACE  (read here to pick core=normal;
                         // the actual per-instruction trace lives in the
                         // submodule and reads getenv() directly.)

  // Strings (nullptr if unset or empty).
  const char* path;      // DOSIZ_PATH
};

extern DebugSettings g_debug;

}  // namespace dosiz
