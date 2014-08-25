#ifndef ORG_EEROS_UI_TUI_HPP_
#define ORG_EEROS_UI_TUI_HPP_

#include <atomic>
#include <deque>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Thread.hpp>

#define INPUT_TIMEOUT 50
#define MAX_NOF_COMMANDS 6
#define HEADER_LINES 3
#define FOOTER_LINES 2
#define STATUS_LINES 4
#define MESSAGE_LINES 6
#define MESSAGE_LINE_MAX_LENGTH 100

namespace eeros {
	namespace ui {
		
		class CursesUI : public eeros::Thread {
			
		public:
			enum State { idle, active, stopping, stopped };
			
			CursesUI(eeros::sequencer::Sequencer& sequencer);
			virtual ~CursesUI();
			
			virtual void dispay();
			virtual void exit();
			
			virtual void addMessage(std::string message);
			
		protected:
			virtual void run();
			
		private:
			eeros::sequencer::Sequencer& sequencer;
			eeros::sequencer::state::type cachedState;
			eeros::sequencer::mode::type cachedMode;
			std::atomic<bool> messageListUpdated;
			std::atomic<State> state;
			unsigned int headerStart;
			unsigned int sequenceListStart;
			unsigned int messageListStart;
			unsigned int statusStart;
			unsigned int commandListStart;
			unsigned int footerStart;
			std::deque<std::string> messageList;
			
			void initScreen();
			void updateScreen();
			void printTitle(std::string text, unsigned int line);
			void printHeader();
			void printFooter(std::string msg);
			void printSequenceList(unsigned int first);
			void printMessageList();
			void printMessage(std::string msg, unsigned int index);
			void printStatus();
			void printCommandList();
			void printCommand(std::string cmd, std::string description, bool active, unsigned int index);
			int promptForInt(std::string message);
// 			double promptForDouble(std::string message);
// 			bool promptConfirm(std::string message);
			
			bool checkCmdToggleIsActive();
			bool checkCmdAbortIsActive();
			bool checkCmdProceedIsActive();
			bool checkCmdChooseSeqIsActive();
			
			static unsigned int instanceCounter;
			
		}; // class TUI
	}; // namespace ui
}; // namespace eeros

#endif // ORG_EEROS_UI_TUI_HPP_
