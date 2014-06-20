#include <eeros/sequencer/TUI.hpp>
#include <curses.h>
#include <sstream>

#define MAX_NOF_COMMANDS 6
#define HEADER_LINES 3
#define FOOTER_LINES 2
#define STATUS_LINES 3

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
	timeout(50);
	
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
	mvprintw(headerStart + 1, 1, "User interface for sequencer #%u", sequencer.getId());
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
		mvprintw(sequenceListStart + 1 + i, 1, "%i. %s", i, entry.second->getName().c_str());
		i++;
	}
}

void TUI::printStatus() {
	printTitle("Status", statusStart);
	mvprintw(statusStart + 1, 1, "Status:");
	Sequencer::status s = sequencer.getStatus();
	switch(s) {
		case Sequencer::automatic:
			mvprintw(statusStart + 1, 9, "running (auto mode)");
			break;
		case Sequencer::stepping:
			mvprintw(statusStart + 1, 9, "stepping (step mode)");
			break;
		case Sequencer::terminating:
			mvprintw(statusStart + 1, 9, "stopping");
			break;
		case Sequencer::terminated:
			mvprintw(statusStart + 1, 9, "stopped");
			break;
		case Sequencer::waiting:
			mvprintw(statusStart + 1, 9, "waiting");
			break;
		default:
			mvprintw(statusStart + 1, 9, "unknown (%u)", static_cast<unsigned int>(s));
			break;
	}
}

void TUI::printCommandList() {
	printTitle("Commands", commandListStart);
	printCommand("F1 ", "display help", true, 0);
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
	printHeader();
	printSequenceList(0);
	printStatus();
	printCommandList();
	printFooter();
	refresh();
}

void TUI::run() {
	unsigned int x = 0;
	while(displayed) {
		mvprintw(LINES - 1, 1, "x = %u", x++);
		
		char c = getch();
		switch(c) {
			case 'h':
				beep();
				break;
			case 't':
				if(sequencer.getStatus() == Sequencer::stepping) sequencer.stepMode(false);
				else if(sequencer.getStatus() == Sequencer::automatic) sequencer.stepMode(true);
				break;
			case 'c':
				if(sequencer.getStatus() == Sequencer::stepping) sequencer.proceed();
				break;
			case 's':
				flash();
				break;
			case 'a':
				if(sequencer.getStatus() == Sequencer::stepping) sequencer.shutdown();
				break;
			case 'e':
				
				sequencer.shutdown();
				exit();
				break;
			default:
				if(sequencer.getStatus() != cachedStatus) {
					updateScreen();
					cachedStatus = sequencer.getStatus();
				}
				break;
		}
	}
}

bool TUI::checkCmdToggleIsActive() {
	return sequencer.getStatus() == Sequencer::automatic || sequencer.getStatus() == Sequencer::stepping;
}

bool TUI::checkCmdAbortIsActive() {
	return sequencer.getStatus() == Sequencer::stepping;
}

bool TUI::checkCmdProceedIsActive() {
	return sequencer.getStatus() == Sequencer::stepping;
}

bool TUI::checkCmdChooseSeqIsActive() {
	return sequencer.getStatus() == Sequencer::waiting;
}
