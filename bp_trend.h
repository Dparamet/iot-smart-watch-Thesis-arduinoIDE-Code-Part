#pragma once

#include <stdint.h>

enum class BpTrend : uint8_t {
  Unknown,
  Normal,
  Check,
  Danger,
};

// Experimental trend only: uses HR/SpO2 because MAX32664 BPT needs calibration.
constexpr BpTrend estimateBpTrend(float heartRate, float spo2) {
  const bool haveHr = heartRate > 0;
  const bool haveSpo2 = spo2 > 0;
  if (!haveHr && !haveSpo2) return BpTrend::Unknown;

  if ((haveHr && (heartRate < 45 || heartRate > 130)) ||
      (haveSpo2 && spo2 < 90)) {
    return BpTrend::Danger;
  }
  if ((haveHr && (heartRate < 55 || heartRate > 100)) ||
      (haveSpo2 && spo2 < 95)) {
    return BpTrend::Check;
  }
  return BpTrend::Normal;
}

constexpr const char* bpTrendLabel(BpTrend trend) {
  switch (trend) {
    case BpTrend::Normal: return "NORMAL";
    case BpTrend::Check:  return "CHECK";
    case BpTrend::Danger: return "DANGER";
    default:              return "--";
  }
}
