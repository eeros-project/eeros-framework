
#include <eeros/core/Executor.hpp>
#include <eeros/core/TimeSource.hpp>
#include <eeros/logger/Logger.hpp>

#include <memory>
#include <iostream>

int main() {
    using namespace eeros::core;
    using namespace eeros::logger;
    std::cout << "start\n";
    Logger::setDefaultStreamLogger(std::cout);
    auto callback = [](TimeSource::Uptime time){
        std::cout << "time: " << time.count() << '\n';
        if (time.count() > 10)
            eeros::Executor::stop();
    };
    std::shared_ptr<TimeSource> clock = std::make_shared<ManualTime>(0.01, callback);
    auto& exec = eeros::Executor::instance();
    exec.setTimeSource(clock);
    exec.run();
}
