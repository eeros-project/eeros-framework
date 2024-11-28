
#include <eeros/core/Executor.hpp>
#include <eeros/core/TimeSource.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/control/Blockio.hpp>

#include <memory>
#include <iostream>
#include <iomanip>

class LongTask : public eeros::Runnable {
public:
    virtual void run() override {
        std::cout << "sleeping...\n";
        std::this_thread::sleep_for(eeros::core::TimeSource::Seconds(0.5));
        std::cout << "...woke up\n";
    }
};

class LogBlock : public eeros::control::Blockio<1, 0> {
    virtual void run() override {
        std::cout << "block time: " << eeros::System::getTime() <<", block signal: " << getIn().getSignal().getValue() << '\n';
    }
};

int main() {
    using namespace eeros::core;
    using namespace eeros::control;
    using namespace eeros::logger;

    constexpr double period = 0.1;
    std::cout << "start\n";
    Logger::setDefaultStreamLogger(std::cout);

    Constant<double> c(1);
    I<double> integrator{};
    integrator.setInitCondition(0);
    LogBlock lb{};
    TimeDomain d("test", period, false);
    d.addBlock(c);
    d.addBlock(integrator);
    d.addBlock(lb);

    integrator.getIn().connect(c.getOut());
    lb.getIn().connect(integrator.getOut());

    integrator.enable();
    d.start();
    // d.run();
    LongTask lt{};
    eeros::task::Periodic p{"long task", 1, static_cast<eeros::Runnable*>(&lt), 1, false};


    auto callback = [&](TimeSource::Uptime time){
        std::cout << std::fixed << std::setprecision(1) << "time: " << std::fixed << time.count() << "\tsignal: " << integrator.getOut().getSignal().getValue() << '\n';
        if (time.count() > 10)
            eeros::Executor::stop();
    };
    std::shared_ptr<TimeSource> clock = std::make_shared<ManualTime>(period, callback);
    auto& exec = eeros::Executor::instance();
    exec.setTimeSource(clock);
    exec.add(d);
    exec.add(p);
    exec.run();
}
