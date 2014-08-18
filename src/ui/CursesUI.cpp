#include <eeros/ui/CursesUI.hpp>
#include <eeros/core/EEROSException.hpp>
#include <curses.h>
#include <sstream>
#include <thread>
#include <chrono>

#define INPUT_TIMEOUT 100
#define MAX_NOF_COMMANDS 6
#define HEADER_LINES 3
#define FOOTER_LINES 2
#define STATUS_LINES 4

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::ui;

unsigned int CursesUI::instanceCounter = 0;

void closeUI() {
	endwin();
}

CursesUI::CursesUI(Sequencer& sequencer) : sequencer(sequencer), state(idle), log(this) {
	if(++instanceCounter > 1) {
		throw EEROSException("Only a single user interface can exist at the same time!");
	}
}

CursesUI::~CursesUI() {
	exit();
}

void CursesUI::dispay() {
	cachedMode = sequencer.getMode();
	cachedState = sequencer.getState();
	
	state = active;
}

void CursesUI::exit() {
	state = stopping;
	clear();
	refresh();
	closeUI();
}

void CursesUI::addMessage(unsigned level, std::string message) {
	// TODO
}

void CursesUI::printHeader() {
	color_set(5, nullptr);
	for(int j = headerStart; j < HEADER_LINES; j++) {
		for(int i = 0; i < COLS; i++) {
			move(j, i);
			addch(' ');
		}
	}
//	attron(A_BOLD);
	std::stringstream title;
	title << "User interface for sequencer " << sequencer.getName();
	mvprintw(headerStart + 1, (COLS - title.str().size()) / 2, "%s", title.str().c_str());
	color_set(1, nullptr);
	attrset(A_NORMAL);
}

void CursesUI::printFooter(std::string msg) {
	color_set(5, nullptr);
	for(int j = 1; j < FOOTER_LINES; j++) {
		for(int i = 0; i < COLS; i++) {
			move(footerStart + j, i);
			addch(' ');
		}
	}
	mvprintw(LINES - 1, 1, "%s", msg.c_str());
	color_set(1, nullptr);
}

void CursesUI::printTitle(std::string text, unsigned int line) {
	color_set(2, nullptr);
	for(int i = 0; i < COLS; i++) {
		move(line, i);
		addch(' ');
	}
	mvprintw(line, (COLS - text.size()) / 2, "%s", text.c_str());
	color_set(1, nullptr);
}

void CursesUI::printSequenceList(unsigned int first) {
// 	const std::vector<Sequence<>*>& list = sequencer.getListOfRegisteredSequences();
// 	unsigned int i = 0;
// 	printTitle("Registered sequences", sequenceListStart);
// 	for(auto entry : list) {
// 		mvprintw(sequenceListStart + 1 + i, 1, "%i. %s (%p)", i, entry->getName().c_str(), entry);
// 		i++;
// 	}
}

void CursesUI::printStatus() {
	printTitle("Status", statusStart);
	mvprintw(statusStart + 1, 1, "State:");
	state::type s = sequencer.getState();
	switch(s) {
		case state::executing:
			mvprintw(statusStart + 1, 9, "executing  ");
			break;
		case state::waiting:
			mvprintw(statusStart + 1, 9, "waiting    ");
			break;
		case state::terminating:
			mvprintw(statusStart + 1, 9, "terminating");
			break;
		case state::terminated:
			mvprintw(statusStart + 1, 9, "terminated ");
			break;
		case state::idle:
			mvprintw(statusStart + 1, 9, "idle       ");
			break;
		default:
			mvprintw(statusStart + 1, 9, "unknown (%u)", static_cast<unsigned int>(s));
			break;
	}
	
	mvprintw(statusStart + 1, COLS / 2 + 1, "Mode:");
	mode::type m = sequencer.getMode();
	switch(m) {
		case mode::automatic:
			mvprintw(statusStart + 1, COLS / 2 + 7, "automatic   ");
			break;
		case mode::stepping:
			mvprintw(statusStart + 1, COLS / 2 + 7, "stepping    ");
			break;
		default:
			mvprintw(statusStart + 1, COLS / 2 + 7, "unknown (%u)", static_cast<unsigned int>(s));
			break;
	}
	
// 	mvprintw(statusStart + 2, 1, "Current Sequence:");
// 	const Sequence<>* cs = sequencer.getCurrentSequence();
// 	move(statusStart + 2, 19); clrtoeol();
// 	if(cs != nullptr) addstr(cs->getName().c_str());
//	printw("%p", cs);
	
}

void CursesUI::printCommandList() {
	printTitle("Commands", commandListStart);
	printCommand("F2 ", "toggle step mode", checkCmdToggleIsActive(), 0);
	printCommand("F5 ", "choose sequence", checkCmdChooseSeqIsActive(), 1);
	printCommand("F3 ", "proceed", checkCmdProceedIsActive(), 2);
	printCommand("F4 ", "abort", checkCmdAbortIsActive(), 4);
	printCommand("F10", "exit", true, 5);
}

void CursesUI::printCommand(std::string cmd, std::string description, bool active, unsigned int index) {
	if(active) color_set(3, nullptr);
	else color_set(4, nullptr);
	
	int line = commandListStart + 1 + index / 2;
	int startCol = 1; if(index % 2 > 0) startCol = COLS / 2 + 1;
	
	mvprintw(line, startCol, cmd.c_str());
	mvprintw(line, startCol + 4, description.c_str());
	color_set(1, nullptr);
}

void CursesUI::updateScreen() {
//	static unsigned int x = 0;
	printHeader();
	printSequenceList(0);
	printStatus();
	printCommandList();
	printFooter("");
//	mvprintw(LINES - 1, 1, "x = %u", x++);
	refresh();
}

void CursesUI::initScreen() {
	initscr();
	atexit(closeUI);
	curs_set(0);
	start_color();
	clear();
	raw();
	keypad(stdscr, TRUE);
	noecho();
	cbreak();
	timeout(INPUT_TIMEOUT);
	
	init_pair(1, COLOR_WHITE, COLOR_BLACK); // Default
	init_pair(2, COLOR_BLACK, COLOR_WHITE);   // Title
	init_pair(3, COLOR_GREEN, COLOR_BLACK); // Available
	init_pair(4, COLOR_RED, COLOR_BLACK);   // Not available
	init_pair(5, COLOR_WHITE, COLOR_RED);   // Header and footer
	
	headerStart = 0;
	footerStart = LINES - FOOTER_LINES;
	sequenceListStart = HEADER_LINES + 1;
	commandListStart = footerStart - MAX_NOF_COMMANDS / 2 - 1;
	statusStart = commandListStart - STATUS_LINES;
}

void CursesUI::run() {
	
	// idle state
	std::chrono::milliseconds sleepDuration(100);
	while(state == idle) {
		std::this_thread::sleep_for(sleepDuration);
	}
	
	// active state
	initScreen();
	while(state == active) {
		int c = getch();
		char input[10];
		switch(c) {
			case KEY_F(2):
				if(checkCmdToggleIsActive()) sequencer.toggleMode();
				updateScreen();
				break;
			case KEY_F(3):
				if(checkCmdProceedIsActive()) sequencer.proceed();
				updateScreen();
				break;
			case KEY_F(4):
				if(checkCmdAbortIsActive()) sequencer.abort();
				updateScreen();
				break;
			case KEY_F(5):
				printFooter("Enter sequence number:");
				nocbreak();
				timeout(-1);
				echo();
				mvgetstr(LINES - 1, 24, input);
				mvprintw(LINES - 1, COLS / 2, "You entered: %i", atoi(input));
//				sequencer.start(atoi(input));
				cbreak();
				noecho();
				timeout(INPUT_TIMEOUT);
				break;
			case 27: // Esc
			case KEY_F(10):
				sequencer.stepMode();
				sequencer.abort();
				while(sequencer.getState() != state::idle);
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
	
	// stopping
	state = stopped;
}

bool CursesUI::checkCmdToggleIsActive() {
	return sequencer.getState() == state::executing || sequencer.getState() == state::waiting;
}

bool CursesUI::checkCmdAbortIsActive() {
	return sequencer.getState() == state::executing || sequencer.getState() == state::waiting;
}

bool CursesUI::checkCmdProceedIsActive() {
	return sequencer.getState() == state::waiting && sequencer.getMode() == mode::stepping;
}

bool CursesUI::checkCmdChooseSeqIsActive() {
	return sequencer.getState() == state::idle && sequencer.getMode() == mode::stepping;
}
