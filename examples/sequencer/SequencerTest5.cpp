#include <iostream>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <functional>
#include <utility>

using namespace eeros::sequencer;
using namespace eeros::logger;

StreamLogWriter w(std::cout);
Logger logger;

class GenericStep: public Step
{
public:
    GenericStep(std::string name, Sequence *caller, std::function<int()> f): Step(std::move(name), caller), function(std::move(f)){}

    int action() override
    {
        return function();
    }

private:
    std::function<int()> function;
};

class MainSequence : public Sequence
{
public:
    MainSequence(std::string name, Sequencer& seq): Sequence(std::move(name), seq) {}

    int action() override
    {
        log.info() << "Hello from an action!";
//         for (auto &step : steps)
//             step();
        GenericStep("1", this, []{logger.info() << "1"; return 0;})();
        GenericStep("2", this, []{logger.info() << "2"; return 0;})();
        return 0;
    }

    void addStep(GenericStep&& step)
    {
        steps.push_back(std::move(step));
    }


private:
    std::vector<GenericStep> steps;
};

int main() {
    logger.set(w);
    Logger::setDefaultWriter(&w);

    logger.info() << "start";

    auto sequencer = Sequencer::instance();
    auto mySequence = MainSequence("main", sequencer);

    mySequence.addStep(GenericStep("1", &mySequence, []{logger.info() << "1"; return 0;}));
    mySequence.addStep(GenericStep("2", &mySequence, []{logger.info() << "2"; return 0;}));

    sequencer.addSequence(mySequence);
    mySequence.start();
    sequencer.wait();
    return 0;
  
}