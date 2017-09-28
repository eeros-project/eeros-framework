#ifndef ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_

#include <eeros/core/Thread.hpp>

namespace eeros {
	namespace sequencer {
				
		void sigPipeHandler(int signum);
		
// 		class Sequencer;

		class SequencerUI : public eeros::Thread {
		public:
			SequencerUI();
			virtual ~SequencerUI();
			
			virtual void stop();
			virtual bool isRunning();
			
		private:
			virtual void run();	
			bool running;
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			int newsockfd;
		};
	};
};

#endif // ORG_EEROS_SEQUENCER_SEQUENCERUI_HPP_