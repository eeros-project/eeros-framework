#include <eeros/control/BlockOutput.hpp>

enum { kSharedMemoryBufLength = 16 };



BlockOutput::BlockOutput()
{
	const char shmPath[] = "/tmp/eeros.shm";
	//std::ofstream shmFileStream(shmPath);
		
	//SharedMemoryWriter writer = SharedMemoryWriter(shmPath, 1, kSharedMemoryBufLength);

}

BlockOutput::~BlockOutput()
{

}

void BlockOutput::run()
{
	dat.timestamp = in.getSignal()->getTimestamp();
	dat.value = in.getSignal()->getValue();
	
	//writer.write(&dat, sizeof(dat));
}