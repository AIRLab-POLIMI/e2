#pragma once

#include "SerialCommunication.h"


class PololuCommunication : public SerialCommunication
{
public:

	PololuCommunication(const char* device = "/dev/ttyACM0");
	~PololuCommunication();

	void WriteActuations() { };
	void ReadEncoders() { };

private:
	void doSomething();
};
