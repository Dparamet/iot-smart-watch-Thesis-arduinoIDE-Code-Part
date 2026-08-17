#pragma once

// Approximate single-cell LiPo state of charge from measured open/load voltage.
constexpr int batteryPercentFromMillivolts(int millivolts) {
  if (millivolts >= 4200) return 100;
  if (millivolts >= 4000) return 78 + (millivolts - 4000) * 22 / 200;
  if (millivolts >= 3800) return 40 + (millivolts - 3800) * 38 / 200;
  if (millivolts >= 3600) return 10 + (millivolts - 3600) * 30 / 200;
  if (millivolts >= 3300) return (millivolts - 3300) * 10 / 300;
  return 0;
}
