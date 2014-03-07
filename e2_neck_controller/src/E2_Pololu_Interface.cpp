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
	printf("about to create PololuCommunication instance...");
	pololuComm = new PololuCommunication( PololuUsbDevice );
	printf(" created!\n\n");
}

void E2_Pololu_Interface::destroyPololuCommunication()
{
	printf("about to destroy PololuCommunication instance...");
	delete pololuComm;
	printf(" destroyed!\n\n");
}

void E2_Pololu_Interface::stopScriptAndGoHome()
{
	pololuComm->Write( (char)StopScript );
	pololuComm->Write( (char)GoHome );
}

void E2_Pololu_Interface::runSubroutine(const int sub)
{
	pololuComm->Write( (char)RunSubroutine );
	pololuComm->Write( (char)sub );

	firstCommand = 0;

	if(	sub == LEDON ) waitForControllerScriptToComplete();
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

	if(	sub == LEDBLINKFASTNSECONDS ) waitForControllerScriptToComplete();
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
	char* c = new char();
//	char myChar = 'A';
	int i = 0;

	printf("waitForControllerScriptToComplete\n");
//	pololuComm->Write( (char)GetScriptStatus );

	printf("--->OK1\n");

	while( i < 1000 )
	{
		pololuComm->Write( (char)GetScriptStatus );
//		pololuComm->Read(&myChar);
		pololuComm->Read(c);
	//	pololuComm->Read(&myInt); // check CS7 in SerialCommunication.cpp

		printf("%d ", static_cast<int>(*c));
//		printf("%d ", static_cast<int>(myChar));

		if (i%70 == 0) printf("\n");

		usleep(100);
		i++;
	}

// interesting flag
// Raw output.
// newtio.c_oflag = 0;

	printf("--->OK2\n");


/*	while( i < 100 )
	{
//		pololuComm->Write( (char)GetScriptStatus );
//		pololuComm->Read(c);

		printf("waiting... %c\n", *c);

		if(c == 0)
			;
		else
			;

		usleep(1000);
		i++;
	}*/
}

void E2_Pololu_Interface::invitationLeft()
{
	waitForControllerScriptToComplete();
	runSubroutine( INVITATIONLEFT );
}

void E2_Pololu_Interface::invitationRight()
{
	waitForControllerScriptToComplete();
	runSubroutine( INVITATIONRIGHT );
}

void E2_Pololu_Interface::give_a_bow()
{
	waitForControllerScriptToComplete();
	runSubroutine( GIVE_A_BOW );
}

void E2_Pololu_Interface::expressSurprise()
{
	waitForControllerScriptToComplete();
	runSubroutine( SURPRISEBEHAVIOUR );
}

void E2_Pololu_Interface::reachStraightPosition()
{
	waitForControllerScriptToComplete();
	runSubroutine( STRAIGHTNECK );
}

void E2_Pololu_Interface::reachStraightNeckPosition()
{
	waitForControllerScriptToComplete();
	runSubroutine( STRAIGHTNECK );
}

void E2_Pololu_Interface::setLowSpeed()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETLOWSPEED );
}

void E2_Pololu_Interface::setNormalSpeed()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETNORMALSPEED );
}

void E2_Pololu_Interface::setHighSpeed()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETHIGHSPEED );
}

void E2_Pololu_Interface::setLowAcc()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETLOWACC );
}

void E2_Pololu_Interface::setNormalAcc()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETNORMALACC );
}

void E2_Pololu_Interface::setHighAcc()
{
	waitForControllerScriptToComplete();
	runSubroutine( SETHIGHACC );
}
