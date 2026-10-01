// Startup-only IMU admission. Samples retain their original timestamps and values.
#ifndef OV_INIT_STARTUPIMUBUFFER_H
#define OV_INIT_STARTUPIMUBUFFER_H
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace ov_init {
template <typename Sample> class StartupImuBuffer {
public:
  bool ready() const { return admitted; }
  const char *reset_reason() const { return reason; }
  double last_gap() const { return gap; }
  std::size_t size() const { return pending.size(); }

  // Before admission, require a continuous window. A clock discontinuity discards
  // the candidate window, so a transient future sample cannot poison EKF buffers.
  std::vector<Sample> push(const Sample &sample, double window_seconds) {
    reason = nullptr;
    if (admitted) return {sample};
    if (!std::isfinite(sample.timestamp) || sample.timestamp < 0.0) {
      pending.clear();
      reason = "invalid_stamp";
      return {};
    }
    if (!pending.empty()) {
      gap = sample.timestamp - pending.back().timestamp;
      if (gap <= 0.0 || gap > 0.1) {
        reason = gap <= 0.0 ? "timestamp_regression" : "timestamp_gap";
        pending.clear();
      }
    }
    pending.push_back(sample);
    if (pending.back().timestamp - pending.front().timestamp < window_seconds) return {};
    admitted = true;
    return std::move(pending);
  }
private:
  bool admitted = false;
  double gap = 0.0;
  const char *reason = nullptr;
  std::vector<Sample> pending;
};
} // namespace ov_init
#endif
