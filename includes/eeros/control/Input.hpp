#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_

class Output;

class Input {
public:
	virtual bool connect(Output& output);
	virtual void disconnect();
	virtual bool isConnected();

protected:
	Output* connectedOutput;
};

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
