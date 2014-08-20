#ifndef ORG_EEROS_SEQUENCER_SEQUENCERESULT_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCERESULT_HPP_

namespace eeros {
	namespace sequencer {
		namespace result {
			enum type { preConditionFailure, postConditionFailure, success };
		}
		
		class SequenceBaseResult {
		public:
			SequenceBaseResult(result::type result) : result(result) { }
			result::type result;
		};
		
		template<typename Treturn>
		class SequenceResult : public SequenceBaseResult {
		public:
			SequenceResult(result::type result) : SequenceBaseResult(result) { }
			SequenceResult(Treturn value) : SequenceBaseResult(result::success), value(value) { }
			SequenceResult(result::type result, Treturn value) : SequenceBaseResult(result), value(value) { }
			
			Treturn value;
		};
		
		template<>
		class SequenceResult<void> : public SequenceBaseResult {
		public:
			SequenceResult(result::type result) : SequenceBaseResult(result) { }
		};
		
	}
}

#endif // ORG_EEROS_SEQUENCER_SEQUENCERESULT_HPP_
