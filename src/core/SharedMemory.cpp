#include <eeros/core/SharedMemory.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>

using namespace eeros;

SharedMemory::SharedMemory(std::string virtualPath, uint32_t size) : virtualPath(virtualPath), size(size) { }

SharedMemory::~SharedMemory() {
	destroy();
}

void* SharedMemory::getMemoryPointer() const {
	return memory;
}

uint32_t SharedMemory::getSize() const {
	return size;
}

int SharedMemory::initialize() {
	fd = shm_open(virtualPath.c_str(), (O_CREAT | O_RDWR), 0600);
	if(fd == -1) return kShmFileOpenError;
//	long pageSize = sysconf(_SC_PAGE_SIZE);
//	while(pageSize < size) pageSize += pageSize;
//	size = pageSize;
	if(ftruncate(fd, (off_t)size)) {
		std::cout << errno;
		return kShmTrucateError;
	}
	memory = mmap(NULL, size, (PROT_READ | PROT_WRITE), (MAP_SHARED), fd, (off_t)0);
	if(memory == MAP_FAILED) return kShmMapError;
	return kShmOk;
}

int SharedMemory::destroy() {
	if(munmap(memory, size)) {
		return kShmUnMapError;
	}
	shm_unlink(virtualPath.c_str());
	return kShmOk;
}