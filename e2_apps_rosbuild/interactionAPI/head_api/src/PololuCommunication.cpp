#include "PololuCommunication.h"

#include <cstring>


PololuCommunication::PololuCommunication()
{
	mModemDevice = strdup("/dev/ttyACM0");

	InitCommunication();
}


PololuCommunication::~PololuCommunication()
{
	CloseCommunication();
}


void PololuCommunication::doSomething()
{

}
