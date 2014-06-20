#include <eeros/sequencer/TUI.hpp>
#include <curses.h>
#include <sstream>

#define MAX_NOF_COMMANDS 6
#define HEADER_LINES 3
#define FOOTER_LINES 2
#define STATUS_LINES 4

using namespace eeros;
using namespace eeros::sequencer;

unsigned int TUI::instanceCounter = 0;

void quit() {
	endwin();
}

TUI::TUI(Sequencer& sequencer) : sequencer(sequencer), displayed(false) {
	if(++instanceCounter > 1) {
		throw EEROSException("Only a single user interface can exist at the same time!");
	}
}

TUI::~TUI() {
	exit();
}

void TUI::dispay() {
	initscr();
	atexit(quit);
	curs_set(0);
	start_color();
	clear();
	noecho();
	cbreak();
	timeout(100);
	
	init_pair(1, COLOR_WHITE, COLOR_BLACK); // Default
	init_pair(2, COLOR_BLACK, COLOR_WHITE);   // Title
	init_pair(3, COLOR_GREEN, COLOR_BLACK); // Available
	init_pair(4, COLOR_RED, COLOR_BLACK);   // Not available
	init_pair(5, COLOR_BLACK, COLOR_BLUE);   // Header and footer
	
	headerStart = 0;
	footerStart = LINES - FOOTER_LINES;
	sequenceListStart = HEADER_LINES + 1;
	commandListStart = footerStart - MAX_NOF_COMMANDS / 2 - 1;
	statusStart = commandListStart - STATUS_LINES;
	
	updateScreen();
	displayed = true;
}

void TUI::exit() {
	displayed = false;
	quit();
}

void TUI::printHeader() {
	color_set(5, nullptr);
	for(int j = headerStart; j < HEADER_LINES; j++) {
		for(int i = 0; i < COLS; i++) {
			move(j, i);
			addch(' ');
		}
	}
	attron(A_BOLD);
	mvprintw(headerStart + 1, 1, "User interface for sequencer ", sequencer.getName().c_str());
	color_set(1, nullptr);
	attrset(A_NORMAL);
}

void TUI::printFooter() {
	color_set(5, nullptr);
	for(int j = 0; j < FOOTER_LINES; j++) {
		for(int i = 0; i < COLS; i++) {
			move(footerStart + j, i);
			addch(' ');
		}
	}
	color_set(1, nullptr);
}

void TUI::printTitle(std::string text, unsigned int line) {
	color_set(2, nullptr);
	for(int i = 0; i < COLS; i++) {
		move(line, i);
		addch(' ');
	}
	mvprintw(line, (COLS - text.size()) / 2, "%s", text.c_str());
	color_set(1, nullptr);
}

void TUI::printSequenceList(unsigned int first) {
	const std::map<std::string, Sequence*>& list = sequencer.getListOfRegisteredSequences();
	unsigned int i = 0;
	printTitle("Registered sequences", sequenceListStart);
	for(auto entry : list) {
		mvprintw(sequenceListStart + 1 + i, 1, "%i. %s (%p)", i, entry.second->getName().c_str(), entry.second);
		i++;
	}
}

void TUI::printStatus() {
	printTitle("Status", statusStart);
	mvprintw(statusStart + 1, 1, "State:");
	Sequencer::State s = sequencer.getState();
	switch(s) {
		case Sequencer::executing:
			mvprintw(statusStart + 1, 9, "executing  ");
			break;
		case Sequencer::waiting:
			mvprintw(statusStart + 1, 9, "waiting    ");
			break;
		case Sequencer::terminating:
			mvprintw(statusStart + 1, 9, "terminating");
			break;
		case Sequencer::terminated:
			mvprintw(statusStart + 1, 9, "terminated ");
			break;
		case Sequencer::idle:
			mvprintw(statusStart + 1, 9, "idle       ");
			break;
		default:
			mvprintw(statusStart + 1, 9, "unknown (%u)", static_cast<unsigned int>(s));
			break;
	}
	
	mvprintw(statusStart + 1, COLS / 2 + 1, "Mode:");
	Sequencer::Mode m = sequencer.getMode();
	switch(m) {
		case Sequencer::automatic:
			mvprintw(statusStart + 1, COLS / 2 + 7, "automatic   ");
			break;
		case Sequencer::stepping:
			mvprintw(statusStart + 1, COLS / 2 + 7, "stepping    ");
			break;
		default:
			mvprintw(statusStart + 1, COLS / 2 + 7, "unknown (%u)", static_cast<unsigned int>(s));
			break;
	}
	
	mvprintw(statusStart + 2, 1, "Current Sequence:");
	const Sequence* cs = sequencer.getCurrentSequence();
	move(statusStart + 2, 19); clrtoeol();
//	if(cs != nullptr) addstr(cs->getName().c_str());
	printw("%p", cs);
	
}

void TUI::printCommandList() {
	printTitle("Commands", commandListStart);
	printCommand("F1 ", "help", true, 0);
	printCommand("F2 ", "toggle step mode", checkCmdToggleIsActive(), 1);
	printCommand("F3 ", "proceed", checkCmdProceedIsActive(), 2);
	printCommand("F5 ", "choose sequence", checkCmdChooseSeqIsActive(), 3);
	printCommand("F9 ", "abort", checkCmdAbortIsActive(), 4);
	printCommand("F10", "exit", true, 5);
}

void TUI::printCommand(std::string cmd, std::string description, bool active, unsigned int index) {
	if(active) color_set(3, nullptr);
	else color_set(4, nullptr);
	
	int line = commandListStart + 1 + index / 2;
	int startCol = 1; if(index % 2 > 0) startCol = COLS / 2 + 1;
	
	mvprintw(line, startCol, cmd.c_str());
	mvprintw(line, startCol + 4, description.c_str());
	color_set(1, nullptr);
}

void TUI::updateScreen() {
	static unsigned int x = 0;
	printHeader();
	printSequenceList(0);
	printStatus();
	printCommandList();
	printFooter();
	mvprintw(LINES - 1, 1, "x = %u", x++);
	refresh();
}

void TUI::run() {
	while(displayed) {
		char c = getch();
		switch(c) {
			case 'h':
				beep();
				break;
			case 't':
				if(checkCmdToggleIsActive()) sequencer.toggleMode();
				break;
			case 'c':
				if(checkCmdProceedIsActive()) sequencer.proceed();
				break;
			case 's':
				flash();
				break;
			case 'a':
				if(checkCmdAbortIsActive()) sequencer.abort();
				break;
			case 'e':
				sequencer.stepMode();
				sequencer.abort();
				sequencer.shutdown();
				exit();
				break;
			default:
				if(sequencer.getState() != cachedState || sequencer.getMode() != cachedMode) {
					updateScreen();
					cachedState = sequencer.getState();
					cachedMode = sequencer.getMode();
				}
				break;
		}
	}
}

bool TUI::checkCmdToggleIsActive() {
	return sequencer.getState() == Sequencer::executing || sequencer.getState() == Sequencer::waiting;
}

bool TUI::checkCmdAbortIsActive() {
	return sequencer.getState() == Sequencer::executing || sequencer.getState() == Sequencer::waiting;
}

bool TUI::checkCmdProceedIsActive() {
	return sequencer.getState() == Sequencer::waiting && sequencer.getMode() == Sequencer::stepping;
}

bool TUI::checkCmdChooseSeqIsActive() {
	return sequencer.getState() == Sequencer::idle && sequencer.getMode() == Sequencer::stepping;
}
