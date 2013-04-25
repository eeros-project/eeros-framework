#include "core/Executor.hpp"
#include "core/ExecutorService.hpp"

void Executor::start() {
	threadId = ExecutorService::createNewThread(this);
}
