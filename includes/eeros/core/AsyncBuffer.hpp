#ifndef EEROS_ASYNC_BUFFER_HPP
#define EEROS_ASYNC_BUFFER_HPP

#include <atomic>
#include <array>

namespace eeros {
    /**
     * AsyncBuffer allows wait free thread safe sharing of values.
     * While designed for a single reader and single writer multiple readers
     * should be fine (see read() below).
     * However, MUTLIPLE WRITERS ARE NOT SAFE!
     * T needs to be default-constructible and copyable
     */
    template<typename T>
	class AsyncBuffer {
		using Index = unsigned char;
		static constexpr Index choose[3][3] = {
			{1, 2, 1},
			{2, 2, 0},
			{1, 0, 1}
		};
		std::array<T, 3> buffer{T{}, T{}, T{}};
		std::atomic<Index> lastReadIndex{0};
		std::atomic<Index> lastWrittenIndex{1};

	public:

	    /**
		 * write new data into buffer
		 * @param data - data to write
		 */
		void write(T data) {
			auto currentRead = lastReadIndex.load(std::memory_order_relaxed);
			auto lastWritten = lastWrittenIndex.load(std::memory_order_relaxed);
			unsigned char writeTo = choose[currentRead][lastWritten];
			buffer[writeTo] = data;
			lastWrittenIndex.store(writeTo, std::memory_order_release);
		}

		/**
		 * read data from buffer
		 * @param newData - out parameter to signal if new data has been written
		 *                  since last read
		 *                  MULTIPLE READERS NOTE: only 1 reader will see this
		 *                  flag set for new data, it is not per reader
		 * @return data currently residing in buffer
		 */
		T read(bool& newData) {
			auto toRead = lastWrittenIndex.load(std::memory_order_acquire);
			newData = toRead == lastReadIndex.load(std::memory_order_relaxed);
			lastReadIndex.store(toRead, std::memory_order_relaxed);
			return buffer[toRead];
		}

		/**
		 * read data from buffer
		 * @return data currently in buffer
		 */
		T read() {
		    bool dummy;
			return read(dummy);
		}
	};
}

#endif
