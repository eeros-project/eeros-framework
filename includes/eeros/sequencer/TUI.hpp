#ifndef ORG_EEROS_SEQUENCER_TUI_HPP_
#define ORG_EEROS_SEQUENCER_TUI_HPP_

#include <atomic>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Thread.hpp>

namespace eeros {
	namespace sequencer {
		
		class TUI : public eeros::Thread {
			
		public:
			TUI(Sequencer& sequencer);
			virtual ~TUI();
			
			virtual void dispay();
			virtual void exit();
			
		protected:
			virtual void run();
			
		private:
			Sequencer& sequencer;
			Sequencer::status cachedStatus;
			std::atomic<bool> displayed;
			unsigned int headerStart;
			unsigned int sequenceListStart;
			unsigned int statusStart;
			unsigned int commandListStart;
			unsigned int footerStart;
			
			void updateScreen();
			void printTitle(std::string text, unsigned int line);
			void printHeader();
			void printFooter();
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
