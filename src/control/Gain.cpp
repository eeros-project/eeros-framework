#include <eeros/control/Gain.hpp>


Gain::Gain(double gain)
{
	this->out = new Output(new AnSignal());
	this->gain = new double[1];
	
	this->gain[0] = gain;
	this->enabled = true;
}

Gain::Gain(AnSignal* signal, double gain)
{
	this->out = new Output(signal);
	this->gain = new double[1];
	
	this->gain[0] = gain;
	this->enabled = true;
}

Gain::Gain(AnSignal* signal, double gain[])
{
	int length = signal->getLength();
	this->out = new Output(signal);
	this->gain = new double[length];
		
	for(int i = 0; i < length; i++)
	{
		this->gain[i] = gain[i];
	}
	this->enabled = true;
}

Gain::~Gain()
{
	delete this->out;
	delete this->gain;
}

void Gain::run()
{
	for(int i = 0; i < out->getSignal()->getLength(); i++)
	{
		if(this->enabled) out->getSignal()->setValue(in.getSignal()->getValue(i) * this->gain[i], i);
		else out->getSignal()->setValue(in.getSignal()->getValue(i), i);
	}
}

void Gain::enable()
{
	this->enabled = true;
}

void Gain::disable()
{
	this->enabled = true;
}

void Gain::setGain(double gain)
{
	this->setGain(gain, 0);
}
	
void Gain::setGain(double gain, int index)
{
	this->gain[index] = gain;
}