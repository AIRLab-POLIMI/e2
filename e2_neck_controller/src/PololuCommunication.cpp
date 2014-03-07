#include "PololuCommunication.h"

#include <cstring>



PololuCommunication::PololuCommunication(const char* device)
{
	mModemDevice = strdup( device );

	InitCommunication();
}


PololuCommunication::~PololuCommunication()
{
	CloseCommunication();
}


void PololuCommunication::doSomething()
{

}
