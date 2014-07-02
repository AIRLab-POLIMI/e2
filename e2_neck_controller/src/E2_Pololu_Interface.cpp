#include "E2_Pololu_Interface.h"


E2_Pololu_Interface::E2_Pololu_Interface()
: PololuUsbDevice("/dev/ttyACM0"), firstCommand(1)
{
	initializePololuCommunication();
}
E2_Pololu_Interface::E2_Pololu_Interface(string device)
: PololuUsbDevice(device.c_str()), firstCommand(1)
{
	initializePololuCommunication();
}

E2_Pololu_Interface::~E2_Pololu_Interface()
{
	destroyPololuCommunication();
}

void E2_Pololu_Interface::initializePololuCommunication()
{
	pololuComm = new PololuCommunication( PololuUsbDevice );
}

void E2_Pololu_Interface::destroyPololuCommunication()
{
	delete pololuComm;
}

void E2_Pololu_Interface::stopScriptAndGoHome()
{
	pololuComm->Write( (char)StopScript );
	pololuComm->Write( (char)GoHome );
}

void E2_Pololu_Interface::stopScript()
{
	pololuComm->Write( (char)StopScript );
}

void E2_Pololu_Interface::runSubroutine(const int sub)
{
	pololuComm->Write( (char)RunSubroutine );
	pololuComm->Write( (char)sub );

	firstCommand = 0;

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

	waitForControllerScriptToComplete();
}

void E2_Pololu_Interface::runSubroutine_nowait(const int sub, const int par)
{
	char parLowBits;
	char parHighBits;

	transformParameter(par, parLowBits, parHighBits);

	pololuComm->Write( (char)RunSubroutineWithParameter );
	pololuComm->Write( (char)sub );
	pololuComm->Write( (char)parLowBits );
	pololuComm->Write( (char)parHighBits );

	firstCommand = 0;
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
	runSubroutine_nowait( sub, firstCommand );

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

		usleep(1000);
	}
	while (c == 0);
}

//==================================================
//	Public Face Functions
//==================================================
void E2_Pololu_Interface::standardFace()
{
	runFaceEmotionalSubroutine(STANDARD_FACE);
}
void E2_Pololu_Interface::happyFace()
{
	runFaceEmotionalSubroutine(HAPPY_FACE);
}
void E2_Pololu_Interface::angryFace()
{
	runFaceEmotionalSubroutine(ANGRY_FACE);
}
void E2_Pololu_Interface::interestedFace()
{
	runFaceEmotionalSubroutine(INTERESTED_FACE);
}
void E2_Pololu_Interface::invitationFace()
{
	runFaceEmotionalSubroutine(INVITATION_FACE);
}
void E2_Pololu_Interface::start_speakingFace()
{
	runFaceSpeakingSubroutine(MOUTH_MOVES);
}
void E2_Pololu_Interface::stop_speakingFace()
{
	stopScript();
}

//==================================================
//	Public Neck Functions
//==================================================
void E2_Pololu_Interface::straightNeck()
{
	runNeckSubroutine( STRAIGHTNECK );
}
void E2_Pololu_Interface::invitationLeft()
{
	runNeckSubroutine( INVITATIONLEFT );
}
void E2_Pololu_Interface::invitationRight()
{
	runNeckSubroutine( INVITATIONRIGHT );
}
void E2_Pololu_Interface::give_a_bow()
{
	runNeckSubroutine( GIVE_A_BOW );
}
void E2_Pololu_Interface::bendForward()
{
	runNeckSubroutine( BENDFORWARD );
}
void E2_Pololu_Interface::bendBack()
{
	runNeckSubroutine( BENDBACK );
}
void E2_Pololu_Interface::bendLeft()
{
	runNeckSubroutine( BENDLEFT );
}
void E2_Pololu_Interface::bendRight()
{
	runNeckSubroutine( BENDRIGHT );
}

//==================================================
//	Public composite Functions
//==================================================

void E2_Pololu_Interface::expressSurprise()
{
	happyFace();
	runFaceEmotionalSubroutine( SURPRISEBEHAVIOUR );
	standardFace();
}

void E2_Pololu_Interface::reachStraightPosition()
{
	runFaceEmotionalSubroutine( STANDARD_FACE );
	runFaceEmotionalSubroutine( STRAIGHTNECK  );
}


//==================================================
//	Public other Functions
//==================================================
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

// Used for test
void E2_Pololu_Interface::blinkLed()
{
	runSubroutine( BLINKSLOW10SECONDS );
}

