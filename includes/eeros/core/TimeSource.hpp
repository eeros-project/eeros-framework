#ifndef ORG_EEROS_CORE_TIMESOURCE_HPP
#define ORG_EEROS_CORE_TIMESOURCE_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

namespace eeros {
namespace core {

/**
 * Interface for timing and scheduling information
 */
class TimeSource {
public:
  using Uptime = std::chrono::duration<double, std::ratio_multiply<std::ratio<1000>, std::chrono::milliseconds::period>>;
  using Seconds = std::chrono::duration<double, std::chrono::seconds::period>;
  /**
   * start time keeping, this should be t=0 for calculating uptime.
   * Also starts the first cycle
   */
  virtual void start() noexcept = 0;
  /**
   * return the elapsed time since start() was called
   */
  virtual Uptime currentUptime() noexcept = 0;
  /**
   * return the period used by the executor.
   * The period should be constant over the lifetime of the executor
   */
  virtual Seconds getPeriod() noexcept = 0;
  /**
   * wait until the next cycle should start
   */
  virtual void sync() = 0;
  virtual ~TimeSource() {}
};


/**
 * Default TimeSource implementation based on the system steady/monotonic clock
 */
class SystemTime : public TimeSource {
  using Clock = std::chrono::steady_clock;
  using TimePoint =
      std::chrono::time_point<std::chrono::steady_clock,
                              std::chrono::duration<double, Clock::period>>;

  TimePoint startPoint;
  TimePoint nextCycle;
  Seconds period;

public:
  /**
   * Create a new SystemTime TimeSource
   *
   * @param period period in seconds
   */
  explicit SystemTime(double period) : period(period) {}

  virtual void start() noexcept override {
    startPoint = Clock::now();
    nextCycle = startPoint + period;
  }
  virtual Uptime currentUptime() noexcept override {
    return Clock::now() - startPoint;
  };

  virtual Seconds getPeriod() noexcept override  {
    return period;
  }
  virtual void sync() override {
    std::this_thread::sleep_until(nextCycle);
    nextCycle += period;
  }
};


/**
 * Simulate a periodic clock with no relation to real world time
 *
 * This
 */
class ManualTime : public TimeSource {
public:
  /**
   * Function to be called on every cycle.
   * Can be used to check if blocks/signals have the expected value
   *
   * @param Uptime the timestamp of the current cycle
   */
  using Callback = std::function<void(Uptime)>;

private:
  Callback callback;
  Seconds period;
  Uptime elapsedTime;

public:
  ManualTime(double period, Callback callback)
      : callback(callback), period(period), elapsedTime(0) {}
  virtual void start() noexcept override {}
  virtual void sync() noexcept override {
    callback(elapsedTime);
    elapsedTime += period;
  }
  virtual Uptime currentUptime() noexcept override {
    return elapsedTime;
  }
  virtual Seconds getPeriod() noexcept override  {
    return period;
  }

};
}; // namespace core
}; // namespace eeros

#endif

