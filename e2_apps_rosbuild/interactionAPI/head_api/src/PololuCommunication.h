
#include "SerialCommunication.h"

using namespace std;


class PololuCommunication : public SerialCommunication
{
public:

	PololuCommunication();

	~PololuCommunication(); // TODO: remove virtual

	void doSomething();

	void WriteActuations() { };
	void ReadEncoders() { };

private:

};
