#include "fairino_hardware/feedback_watchdog.hpp"
#include <limits>
#include <stdexcept>

void require(bool condition) {
  if (!condition) throw std::runtime_error("Feedback watchdog regression");
}
int main() {
  fairino_hardware::FeedbackWatchdog guard;
  require(guard.observe(254, true, 1.0));
  require(guard.observe(254, true, 1.09));
  require(!guard.observe(254, true, 1.101));
  require(!guard.observe(255, true, 1.102));
  guard.reset();
  require(guard.observe(255, true, 2.0));
  require(guard.observe(0, true, 2.008));
  require(guard.observe(9, true, 2.080));
  require(!guard.observe(10, false, 2.088));
  guard.reset();
  require(!guard.observe(0, true, std::numeric_limits<double>::quiet_NaN()));
  guard.reset();
  require(guard.observe(0, true, 3.0));
  require(!guard.observe(1, true, 2.0));
}
