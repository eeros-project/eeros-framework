
#include <eeros/core/Executor.hpp>
#include <eeros/core/TimeSource.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/System.hpp>

#include <memory>
#include <iostream>

int main() {
    using namespace eeros::core;
    using namespace eeros::control;
    using namespace eeros::logger;
    std::cout << "start\n";
    Logger::setDefaultStreamLogger(std::cout);

    Constant<double> c(1);
    I<double> integrator{};
    integrator.setInitCondition(0);
    TimeDomain d("test", 1, false);
    d.addBlock(c);
    d.addBlock(integrator);

    integrator.getIn().connect(c.getOut());

    integrator.enable();
    d.start();
    // d.run();


    auto callback = [&](TimeSource::Uptime time){
        std::cout << "time: " << time.count() << "\tsignal: " << integrator.getOut().getSignal().getValue() << '\n';
        if (time.count() > 10)
            eeros::Executor::stop();
    };
    std::shared_ptr<TimeSource> clock = std::make_shared<ManualTime>(1, callback);
    auto& exec = eeros::Executor::instance();
    exec.setTimeSource(clock);
    exec.add(d);
    exec.run();
}
