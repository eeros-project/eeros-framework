#include <eeros/core/RingBuffer.hpp>
#include <string.h>

RingBuffer::RingBuffer(void* memory, uint32_t size) {
	char* pWork = static_cast<char*>(memory);
	ringSize = size - sizeof(*pWriteIndex);
	pWriteIndex = static_cast<uint32_t*>(memory);
	if(*pWriteIndex < 0 || *pWriteIndex >= ringSize) {
		*pWriteIndex = 0;
	}
	ring = pWork + sizeof(*pWriteIndex);
 	pWrite = ring + *pWriteIndex;
	pRead = pWrite;
}

uint32_t RingBuffer::avalableToRead() const {
	char* pWrite = ring + *pWriteIndex;
	if(pRead == pWrite) {
		return 0;
	}
	else if(pWrite > pRead) {
		return pWrite - pRead;
	}
	else {
		return ((ring + ringSize) - pRead) + (pWrite - ring);
	}
}

uint32_t RingBuffer::read(void* pDest, uint32_t size) {
	uint32_t available =  avalableToRead();
	pWrite = ring + *pWriteIndex;
	if(available && size <= available) {
		if(pRead + size < ring + ringSize) { // read in a single step
			memcpy(pDest, pRead, size);
			pRead += size;
		}
		else { // read in tow steps
			uint32_t sizeToCopy = ring + ringSize - pRead;
			memcpy(pDest, pRead, sizeToCopy);
			pRead = ring;
			
			char* pDest2 = static_cast<char*>(pDest) + sizeToCopy;
			sizeToCopy = size - sizeToCopy;
			memcpy(pDest2, pRead, sizeToCopy);
			pRead += sizeToCopy;
		}
	}
	return available;
}

uint32_t RingBuffer::write(void* pSrc, uint32_t size) {
	if(pWrite + size < ring + ringSize) { // write in a single step
		memcpy(pWrite, pSrc, size);
		pWrite += size;
	}
	else { // write in two steps
		uint32_t sizeToCopy = ring + ringSize - pWrite;
		memcpy(pWrite, pSrc, sizeToCopy);
		pWrite = ring;
		
		char* pSrc2 = static_cast<char*>(pSrc) + sizeToCopy;
		sizeToCopy = size - sizeToCopy;
		memcpy(pWrite, pSrc2, sizeToCopy);
		pWrite += sizeToCopy;
	}
	*pWriteIndex = pWrite - ring;
	return size;
}

uint32_t RingBuffer::size() const {
	return ringSize;
}