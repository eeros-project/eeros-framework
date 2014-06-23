#ifndef ORG_EEROS_SEQUENCER_TUI_HPP_
#define ORG_EEROS_SEQUENCER_TUI_HPP_

#include <atomic>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Thread.hpp>

namespace eeros {
	namespace sequencer {
		
		class TUI : public eeros::Thread {
			
		public:
			enum State { idle, active, stopping, stopped };
			
			TUI(Sequencer& sequencer);
			virtual ~TUI();
			
			virtual void dispay();
			virtual void exit();
			
		protected:
			virtual void run();
			
		private:
			Sequencer& sequencer;
			Sequencer::State cachedState;
			Sequencer::Mode cachedMode;
			std::atomic<State> state;
			unsigned int headerStart;
			unsigned int sequenceListStart;
			unsigned int statusStart;
			unsigned int commandListStart;
			unsigned int footerStart;
			
			void initScreen();
			void updateScreen();
			void printTitle(std::string text, unsigned int line);
			void printHeader();
			void printFooter(std::string msg);
			void printSequenceList(unsigned int first);
			void printStatus();
			void printCommandList();
			void printCommand(std::string cmd, std::string description, bool active, unsigned int index);
			
			bool checkCmdToggleIsActive();
			bool checkCmdAbortIsActive();
			bool checkCmdProceedIsActive();
			bool checkCmdChooseSeqIsActive();
			
			static unsigned int instanceCounter;
			
		}; // class TUI
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_TUI_HPP_
