#ifndef ORG_EEROS_CORE_SHAREDMEMORY_HPP_
#define ORG_EEROS_CORE_SHAREDMEMORY_HPP_

#include <string>
#include <eeros/types.hpp>

namespace eeros {

	enum SharedMemoryError { kShmFileOpenError = -1, kShmTrucateError = -2, kShmMapError = -3, kShmUnMapError = -4 };
	enum SharedMemoryError1 { kShmOk = 0, kShmError = -1};

	class SharedMemory {
	public:
		SharedMemory(std::string virtualPath, uint32_t size);
		virtual ~SharedMemory();
		
		virtual void* getMemoryPointer() const;
		virtual uint32_t getSize() const;
		virtual int initialize();
		virtual int destroy();
		
	private:
		std::string virtualPath;
		uint32_t size;
		int fd;
		void* memory;
	};

};

#endif // ORG_EEROS_CORE_SHAREDMEMORY_HPP_
