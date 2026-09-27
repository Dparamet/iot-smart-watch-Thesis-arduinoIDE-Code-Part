#include "bp_trend.h"
constexpr bool sameText(const char* left, const char* right) {
  return (*left == *right) &&
         (*left == '\0' || sameText(left + 1, right + 1));
}

static_assert(estimateBpTrend(0, 0) == BpTrend::Unknown);
static_assert(estimateBpTrend(72, 98) == BpTrend::Normal);
static_assert(estimateBpTrend(105, 98) == BpTrend::Check);
static_assert(estimateBpTrend(72, 93) == BpTrend::Check);
static_assert(estimateBpTrend(135, 98) == BpTrend::Danger);
static_assert(estimateBpTrend(72, 88) == BpTrend::Danger);
static_assert(sameText(bpTrendLabel(BpTrend::Unknown), "--"));
static_assert(sameText(bpTrendLabel(BpTrend::Normal), "NORMAL"));
static_assert(sameText(bpTrendLabel(BpTrend::Check), "CHECK"));
static_assert(sameText(bpTrendLabel(BpTrend::Danger), "DANGER"));

int main() {
  return 0;
}
