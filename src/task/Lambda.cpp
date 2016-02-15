#include <eeros/task/Lambda.hpp>

using namespace eeros::task;

Lambda::Lambda() : f([](){}) { }
Lambda::Lambda(std::function<void()> f) : f(f) { }
void Lambda::run() { f(); }
