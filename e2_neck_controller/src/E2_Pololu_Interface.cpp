#include "E2_Pololu_Interface.h"


E2_Pololu_Interface::E2_Pololu_Interface()
: PololuUsbDevice("/dev/ttyACM0"), firstCommand(1)
{
	initializePololuCommunication();
}

E2_Pololu_Interface::~E2_Pololu_Interface()
{
	destroyPololuCommunication();
}

void E2_Pololu_Interface::initializePololuCommunication()
{
	//printf("about to create PololuCommunication instance...");
	pololuComm = new PololuCommunication( PololuUsbDevice );
	//printf(" created!\n\n");
}

void E2_Pololu_Interface::destroyPololuCommunication()
{
	//printf("about to destroy PololuCommunication instance...");
	delete pololuComm;
	//printf(" destroyed!\n\n");
}

void E2_Pololu_Interface::stopScriptAndGoHome()
{
	pololuComm->Write( (char)StopScript );
	pololuComm->Write( (char)GoHome );
}

void E2_Pololu_Interface::runSubroutine(const int sub)
{
//	waitForControllerScriptToComplete();

	pololuComm->Write( (char)RunSubroutine );
	pololuComm->Write( (char)sub );

	firstCommand = 0;

//	if(	sub == LEDON || sub == BLINKSLOW10SECONDS)
	waitForControllerScriptToComplete();
}

void E2_Pololu_Interface::runSubroutine(const int sub, const int par)
{
	char parLowBits;
	char parHighBits;

	transformParameter(par, parLowBits, parHighBits);

	pololuComm->Write( (char)RunSubroutineWithParameter );
	pololuComm->Write( (char)sub );
	pololuComm->Write( (char)parLowBits );
	pololuComm->Write( (char)parHighBits );

	firstCommand = 0;

//	if(	sub == LEDBLINKFASTNSECONDS || sub == LEDBLINKSLOWNSECONDS)
	waitForControllerScriptToComplete();
}

void E2_Pololu_Interface::runNeckSubroutine(const int sub)
{
	runSubroutine( sub, firstCommand );

	firstCommand = 0;
}

void E2_Pololu_Interface::runFaceEmotionalSubroutine(const int sub)
{
	runSubroutine( sub, firstCommand );

	firstCommand = 0;
}

void E2_Pololu_Interface::runFaceSpeakingSubroutine(const int sub)
{
	runSubroutine( sub, firstCommand );

	firstCommand = 0;
}


void E2_Pololu_Interface::transformParameter(const int parameter, char& parameterLowbits, char& parameterHighBits)
{
	parameterLowbits  = parameter & 0x7F;
	parameterHighBits = (parameter >> 7) & 0x7F;
}

void E2_Pololu_Interface::waitForControllerScriptToComplete()
{
	char c;

	do
	{
		pololuComm->Write( (char)GetScriptStatus );
		pololuComm->Read(&c);

//		printf("[%d]\n", c);

		usleep(1000);
	}
	while (c == 0);
}


// PUBLIC INTERFACE
void E2_Pololu_Interface::invitationLeft()
{
	runSubroutine( INVITATIONLEFT );
}

void E2_Pololu_Interface::invitationRight()
{
	runSubroutine( INVITATIONRIGHT );
}

void E2_Pololu_Interface::give_a_bow()
{
	runSubroutine( GIVE_A_BOW );
}

void E2_Pololu_Interface::expressSurprise()
{
	runSubroutine( SURPRISEBEHAVIOUR );
}

void E2_Pololu_Interface::reachStraightPosition()
{
	runSubroutine( STRAIGHTNECK );
}

void E2_Pololu_Interface::reachStraightNeckPosition()
{
	runSubroutine( STRAIGHTNECK );
}

void E2_Pololu_Interface::setLowSpeed()
{
	runSubroutine( SETLOWSPEED );
}

void E2_Pololu_Interface::setNormalSpeed()
{
	runSubroutine( SETNORMALSPEED );
}

void E2_Pololu_Interface::setHighSpeed()
{
	runSubroutine( SETHIGHSPEED );
}

void E2_Pololu_Interface::setLowAcc()
{
	runSubroutine( SETLOWACC );
}

void E2_Pololu_Interface::setNormalAcc()
{
	runSubroutine( SETNORMALACC );
}

void E2_Pololu_Interface::setHighAcc()
{
	runSubroutine( SETHIGHACC );
}

void E2_Pololu_Interface::bendForward()
{
	runSubroutine( BENDFORWARD );
}

void E2_Pololu_Interface::bendBack()
{
	runSubroutine( BENDBACK );
}

void E2_Pololu_Interface::bendLeft()
{
	runSubroutine( BENDLEFT );
}

void E2_Pololu_Interface::bendRight()
{
	runSubroutine( BENDRIGHT );
}
