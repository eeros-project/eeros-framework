#ifndef ORG_EEROS_UI_TUI_HPP_
#define ORG_EEROS_UI_TUI_HPP_

#include <atomic>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/logger/UILogWriter.hpp>
#include <eeros/ui/BaseUI.hpp>

namespace eeros {
	namespace ui {
		
		class CursesUI : public eeros::Thread, public BaseUI {
			
		public:
			enum State { idle, active, stopping, stopped };
			
			CursesUI(eeros::sequencer::Sequencer& sequencer);
			virtual ~CursesUI();
			
			virtual void dispay();
			virtual void exit();
			
			virtual void addMessage(unsigned level, std::string message);
			
		protected:
			virtual void run();
			
		private:
			eeros::logger::UILogWriter log;
			eeros::sequencer::Sequencer& sequencer;
			eeros::sequencer::Sequencer::State cachedState;
			eeros::sequencer::Sequencer::Mode cachedMode;
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
	}; // namespace ui
}; // namespace eeros

#endif // ORG_EEROS_UI_TUI_HPP_
