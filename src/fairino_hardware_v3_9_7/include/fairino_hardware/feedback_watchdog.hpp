#pragma once
#include <cmath>
#include <cstdint>

namespace fairino_hardware {
// A changing controller frame, not a new ROS timestamp, proves freshness.
class FeedbackWatchdog {
public:
  bool observe(std::uint8_t frame, bool healthy, double now) {
    if (faulted_ || !healthy || !std::isfinite(now) || (seen_ && now < changed_)) {
      faulted_ = true;
      return false;
    }
    if (!seen_ || frame != frame_) {
      frame_ = frame;
      changed_ = now;
      seen_ = true;
    }
    if (now - changed_ > 0.1) faulted_ = true;
    return !faulted_;
  }
  void reset() { *this = FeedbackWatchdog{}; }
private:
  std::uint8_t frame_ = 0;
  double changed_ = 0.0;
  bool seen_ = false;
  bool faulted_ = false;
};
}  // namespace fairino_hardware
