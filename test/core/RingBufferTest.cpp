#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <iostream>
#include <eeros/core/RingBuffer.hpp>

using namespace eeros;
using namespace std;

RingBuffer<int, 4> rb;

int i = 0;
int j = 0;
int v;
unsigned int ERROR = 0;

#define PUSH() push(__LINE__)
#define POP() pop(__LINE__)
#define PUSH_FAIL() push_fail(__LINE__)
#define POP_FAIL() pop_fail(__LINE__)
#define LEN(x) len(x, __LINE__)


void error(int line, eeros::RingBuffer<int, 4>& rb) {
	int head = ((unsigned int*)((void*)&rb))[0];
	int tail = ((unsigned int*)((void*)&rb))[1];
	unsigned int *items = &((unsigned int*)((void*)&rb))[2];
	
	std::cout << "		<<< ERROR	LINE: " << line;
	std::cout << " LEN: " << rb.length();
	std::cout << " HEAD: " << head;
	std::cout << " TAIL: " << tail;
	std::cout << "     ";
	for(int i = 0; i < rb.size(); i++) 
		std::cout << " " << items[i];
	
	ERROR++;
}

void push(int line) {
	cout << "push " << i;
	if(!rb.push(i++)) error(line, rb);
	std::cout << std::endl;
}

void push_fail(int line) {
	if(rb.push(i)) 	{
		cout << "push fail " << i;
		error(line, rb);
		std::cout << std::endl;
	}
}

void pop(int line) {
	cout << "pop " << j;
	if(!rb.pop(v) || v != j++) error(line, rb);
	std::cout << std::endl;
}

void pop_fail(int line) {
	if(rb.pop(v)) {
		cout << "pop fail " << j;
		error(line, rb);
		std::cout << std::endl;
	}
}

void len(int len, int line) {
	cout << "len " << len;
	if(rb.length() != (uint32_t)len) error(line, rb);
	std::cout << std::endl;
}

int main() {
	
	LEN(0);
	PUSH();	LEN(1);
	PUSH();	LEN(2);
	PUSH();	LEN(3);
	PUSH();	LEN(4);
	POP();	LEN(3);
	POP();	LEN(2);
	POP();	LEN(1);
	PUSH();	LEN(2);
	PUSH();	LEN(3);
	PUSH();	LEN(4);
	PUSH_FAIL();
	POP();	LEN(3);
	POP();	LEN(2);
	POP();	LEN(1);
	POP();	LEN(0);
	POP_FAIL();
	
	if(ERROR > 0) {
		cout << "Test failed with " << ERROR << " error(s)!" << endl;
	}
	else {
		cout << "Test passed!" << endl;
	}
	
	return ERROR;
}
