#include "battery_level.h"

static_assert(batteryPercentFromMillivolts(4200) == 100);
static_assert(batteryPercentFromMillivolts(4000) == 78);
static_assert(batteryPercentFromMillivolts(3800) == 40);
static_assert(batteryPercentFromMillivolts(3600) == 10);
static_assert(batteryPercentFromMillivolts(3300) == 0);

int main() {
  return 0;
}
