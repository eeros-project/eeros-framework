#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <thread>

/*
* example with several harmonic tasks
*/ 
using namespace eeros::logger;
using namespace eeros;

int main() {
  const double dt = 1.0;
  logger::StreamLogWriter w(std::cout);
  Logger::setDefaultWriter(&w);
  w.show();

  Logger log('M');
  Logger log2('T');

  log.trace() << "harmonic tasks example 3";

  Executor& executor = Executor::instance();

  task::Lambda lmainTask ([&] () {
    log.info() << "run main task at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 2; i++) {
      log.info() << "\tdo some work in main task";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic mainTask("main task", dt, lmainTask);
  executor.setMainTask(mainTask);
  mainTask.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "mainTask: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });

  task::Lambda t1 ([&] () {
    log2.info() << "run task1 at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 4; i++) {
      log2.info() << "\tdo some work in task1";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic task1("task1", dt, t1);
  task1.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "task1: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });
  
  task::Lambda t2 ([&] () {
    log2.info() << "run task2 at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 3; i++) {
      log2.info() << "\tdo some work in task2";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic task2("task2", 2 * dt, t2);
  task2.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "task2: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });

  task::Lambda t4 ([&] () {
    log2.info() << "run task4 at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 2; i++) {
      log2.info() << "\tdo some work in task4";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic task4("task4", 4 * dt, t4);
  task2.after.push_back(task4);
  task1.after.push_back(task2);
  task4.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "task4: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });

  task::Lambda t3 ([&] () {
    log2.info() << "run task3 at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 4; i++) {
      log2.info() << "\tdo some work in task3";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic task3("task3", 5 * dt, t3);
  task1.after.push_back(task3);
  task3.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "task3: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });

  task::Lambda t5 ([&] () {
    log2.info() << "run task5 at timestamp " << (long)System::getTimeNs();
    for (int i = 0; i < 6; i++) {
      log2.info() << "\tdo some work in task5";
      std::this_thread::sleep_for(std::chrono::microseconds((int)(dt * 200000)));
    }
  });
  task::Periodic task5("task5", 3 * dt, t5);
  task5.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    static int ticks = 0;
    if (++ticks < 10) return;
    ticks = 0;
    log.info() << "task5: period max: " << c.period.max << "   run max: " << c.run.max << "   run mean: " << c.run.mean;
    c.reset();
  });

  executor.add(task1);
  executor.add(task5);

  executor.run();

  return 0;
}
