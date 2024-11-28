
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
#include <functional>
#include <initializer_list>
#include <cassert>

class LongTask : public eeros::Runnable {
public:
    virtual void run() override {
        std::cout << "sleeping...\n";
        std::this_thread::sleep_for(eeros::core::TimeSource::Seconds(0.5));
        std::cout << "...woke up\n";
    }
};

template<typename T>
class LogBlock : public eeros::control::Blockio<1, 0, T> {
    virtual void run() override {
        std::cout << "block time: " << eeros::System::getTime() <<", block signal: " << this->getIn().getSignal().getValue() << '\n';
    }
};

struct ReferenceGenerator {
    using Uptime = eeros::core::SystemTime::Uptime;
    using GeneratorFunc = std::function<double(Uptime)>;
    Uptime end;
    GeneratorFunc generator;
};

class Validator : public eeros::control::Blockio<1, 1, double, bool> {
    public:
        Validator(std::initializer_list<ReferenceGenerator> references): generators(references) {
            assert(generators.size() > 0);
        }

        std::size_t failures() {
            return failedCycles;
        }
private:
    std::vector<ReferenceGenerator> generators;
    std::size_t currentGeneratorIndex = 0;
    ReferenceGenerator::Uptime generatorStart;
    std::size_t failedCycles = 0;

    virtual void run() override {
        auto now = eeros::Executor::uptime();
        while(currentGeneratorIndex < generators.size() && generators[currentGeneratorIndex].end < now) {
            generatorStart = generators[currentGeneratorIndex].end;
            ++currentGeneratorIndex;
        };
        auto& in = getIn().getSignal();
        auto& out = getOut().getSignal();
        auto result = std::abs(in.getValue() - generators[currentGeneratorIndex].generator(now - generatorStart)) < 1e-6;
        failedCycles += !result; //branchless version of if(!result) ++failedCycles;
        out.setValue(result);
        out.setTimestamp(eeros::System::getTimeNs());
    }
};

int main() {
    using namespace eeros::core;
    using namespace eeros::control;
    using namespace eeros::logger;

    constexpr double period = 0.1;
    std::cout << std::boolalpha <<"start\n";
    Logger::setDefaultStreamLogger(std::cout);

    Constant<double> c(1);
    I<double> integrator{};
    integrator.setInitCondition(0);
    Validator v{ReferenceGenerator{ReferenceGenerator::Uptime(11), [](ReferenceGenerator::Uptime time){return time.count() - 0.1;}}};
    LogBlock<bool> lb{};
    TimeDomain d("test", period, false);
    d.addBlock(c);
    d.addBlock(integrator);
    d.addBlock(v);
    d.addBlock(lb);

    integrator.getIn().connect(c.getOut());
    v.getIn().connect(integrator.getOut());
    lb.getIn().connect(v.getOut());

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
    std::cout << "failed validations: " << v.failures() << '\n';
}

