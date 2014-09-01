#ifndef ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_
#define ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_

#include <array>

namespace eeros {
	namespace control {
		
		template<typename T, int N = 1>
		class TrajectoryGenerator {
			
		public:
			TrajectoryGenerator() {
				for(auto& e : last) {
					e = 0;
				}
			}
			
			virtual bool finished() = 0;
			
			virtual std::array<T, N> get(double dt) = 0;
			
			virtual bool push(std::array<T, N> start, std::array<T, N> end) = 0;
			
			virtual bool push(T end) {
				std::array<T, N> e;
				for(auto& i : e) i = 0;
				e[0] = end;
				return push(last, e);
			}
			
			virtual bool push(std::array<T, N> end) {
				return push(last, end);
			}
			
			virtual bool push(T start, T end) {
				std::array<T, N> s, e;
				for(auto& i : e) i = 0; for(auto& i : s) i = 0;
				s[0] = start; e[0] = end;
				return push(s, e);
			}
			
			virtual void reset(std::array<T, N> last) = 0;
			
		protected:
			std::array<T, N> last;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_TRAJECTORYGENERATOR_HPP_ */

