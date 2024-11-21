#ifndef ORG_EEROS_CORE_TIMESOURCE_HPP
#define ORG_EEROS_CORE_TIMESOURCE_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

namespace eeros {
namespace core {

class TimeSource {
public:
  using Uptime = std::chrono::duration<double, std::ratio_multiply<std::ratio<1000>, std::chrono::milliseconds::period>>;
  using Seconds = std::chrono::duration<double, std::chrono::seconds::period>;
  virtual void start() noexcept = 0;
  virtual Uptime currentUptime() noexcept = 0;
  virtual Seconds getPeriod() noexcept = 0;
  virtual void sync() = 0;
  virtual ~TimeSource() {}
};

class SystemTime : public TimeSource {
  using Clock = std::chrono::steady_clock;
  using TimePoint =
      std::chrono::time_point<std::chrono::steady_clock,
                              std::chrono::duration<double, Clock::period>>;

  TimePoint startPoint;
  TimePoint nextCycle;
  Seconds period;

public:
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

class ManualTime : public TimeSource {
public:
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
