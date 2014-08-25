#ifndef ORG_EEROS_CORE_SIGNALBUFFER_HPP_
#define ORG_EEROS_CORE_SIGNALBUFFER_HPP_

#include <stdint.h>
#include <array>
#include <mutex>

namespace eeros {

	template<typename T, int N = 32>
	class RingBuffer {
	public:
		RingBuffer() : tail(0), len(0) { }
		
		bool push(T v) {
			std::lock_guard<std::mutex> lock(mtx);
			if(len == N) return false;
			items[(tail + len++) % N] = v;
			return true;
		}
		
		bool pop(T& v) {
			std::lock_guard<std::mutex> lock(mtx);
			if(len == 0) return false;
			v = items[tail];
			tail = (tail + 1) % N;
			len--;
			return true;
		}
		
		unsigned int length() const { return len; }
		
		constexpr int size() const { return N; }
		
	private:
		std::mutex mtx;
		unsigned int tail; // points to the next readable item
		unsigned int len;
		T items[N];
	};
};

#endif // ORG_EEROS_CORE_SIGNALBUFFER_HPP_
