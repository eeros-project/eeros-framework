#ifndef ORG_EEROS_TASK_PERIODIC_HPP_
#define ORG_EEROS_TASK_PERIODIC_HPP_

#include <vector>
#include <functional>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>

namespace eeros {
namespace task {

/**
 * A periodic is used to be run by the @ref Executor. 
 * All periodics must be harmonic. That is, their periods must be a integral multiple the base periodic.
 *  
 * @since v0.4
 */
class Periodic {
 public:
  /**
   * Constructs a periodic instance. Upon installation in the @ref Executor the runnable object will
   * be run with the given period.
   *
   * @param name - name of the periodic
   * @param period - period with which the periodic will run
   * @param task - the runnable the periodic will actually run
   * @param realtime - if true, the excutor will allocate a realtime priority to the associated thread
   * @param nice - nice level of the associated thread
   */
  Periodic(const std::string name, double period, Runnable &task, bool realtime = true, int nice = -1)
      : name(name), period(period), task(&task), realtime(realtime), nice(nice) { }
      
  /**
   * Constructs a periodic instance. Upon installation in the @ref Executor the runnable object will
   * be run with the given period.
   *
   * @param name - name of the periodic
   * @param period - period with which the periodic will run
   * @param task - the runnable the periodic will actually run
   * @param realtime - if true, the excutor will allocate a realtime priority to the associated thread
   * @param nice - nice level of the associated thread
   */
  Periodic(const std::string name, double period, Runnable *task, bool realtime = true, int nice = -1)
      : name(name), period(period), task(task), realtime(realtime), nice(nice) { }
      
  /**
   * You can a default monitor to a periodic. Such a monitor will will log a message (on level WARN)
   * as soon as the maximum period exceeds a the envisaged period more than a certain limit. 
   * The default value for this limit is 5%.
   *
   * @param tolerance - the amount the actual period can vary compared to its chosen period
   */
  void addDefaultMonitor(double tolerance = 0.05) {
    PeriodicCounter::addDefaultMonitor(monitors, period, tolerance);
  }

  /**
   * Gets the name of the periodic.
   * 
   * @return name
   */
  std::string getName() {
    return name;
  }
  
  /**
   * Gets the period of the periodic.
   * 
   * @return period
   */
  double getPeriod() {
    return period;
  }
  
  /**
   * Gets the runnable of the periodic.
   * 
   * @return task
   */
  Runnable& getTask() {
    return *task;
  }

  /**
   * Gets the realtime flag of the periodic.
   * 
   * @return realtime
   */
  bool getRealtime() {
    return realtime;
  }  
  
  /**
   * Gets the nice level of the periodic.
   * 
   * @return nice
   */
  int getNice() {
    return nice;
  }
  
  /**
   * Sets the nice level of the periodic.
   * 
   * @param value nice level
   */
  void setNice(int value) {
    nice = value;
  }

  /**
   * A periodic can be chosen to be run before another periodic.
   * In such a case you have to add it to this vector.
   */
  std::vector<Periodic> before;
  /**
   * A periodic can be chosen to be run after another periodic.
   * In such a case you have to add it to this vector.
   */
  std::vector<Periodic> after;

  /**
   * A periodic can have monitors. Monitors can be added with the function 
   * push_back(). Add the monitor function before adding the 
   * periodic task to the executor. 
   */
  std::vector<PeriodicCounter::MonitorFunc> monitors;

 private:
  std::string name;
  double period;
  Runnable *task;
  bool realtime;
  int nice;
};

}
}

#endif // ORG_EEROS_TASK_PERIODIC_HPP_
