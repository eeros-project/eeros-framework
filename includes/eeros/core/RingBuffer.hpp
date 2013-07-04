#ifndef ORG_EEROS_CORE_SIGNALBUFFER_HPP_
#define ORG_EEROS_CORE_SIGNALBUFFER_HPP_
#include <stdint.h>

class RingBuffer {
public:
	RingBuffer(void* memory, uint32_t size);
	
	uint32_t read(void* pDest, uint32_t size);
	uint32_t write(void* pSrc, uint32_t size);
	uint32_t avalableToRead() const;
	uint32_t size() const;
	

private:
	uint32_t ringSize;
	uint32_t* pWriteIndex;
		
	char* ring;
 	char* pRead;
	char* pWrite;
};

#endif // ORG_EEROS_CORE_SIGNALBUFFER_HPP_
