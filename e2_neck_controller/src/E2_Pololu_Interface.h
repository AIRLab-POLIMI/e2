#pragma once

#include <iostream>
#include <cstdio>

#include "PololuCommunication.h"

class E2_Pololu_Interface
{
public:
	E2_Pololu_Interface();
	~E2_Pololu_Interface();

	const char* PololuUsbDevice;
	char firstCommand;
	PololuCommunication* pololuComm;

	enum commands
	{
		GetMovingState  = 0x93,
		GoHome          = 0xA2, // 162,
		StopScript      = 0xA4, // 164,
		GetScriptStatus = 0xAE, // 174,
		RunSubroutine   = 0xA7, // 167,
		RunSubroutineWithParameter = 0xA8 // 168
	};

	enum subroutines
	{
//		Face - emotional expressions
		STANDARD_FACE     = 0x00,
		HAPPY_FACE        = 0x01,
		ANGRY_FACE        = 0x02,
		INTERESTED_FACE   = 0x03,
		INVITATION_FACE   = 0x04,
//		Face - speaking
		MOUTH_MOVES       = 0x05,

//		Neck Subroutines
		GIVE_A_BOW        = 0x06,
		STRAIGHT          = 0x07,
		STRAIGHTNECK      = 0x08,
		INVITATIONLEFT    = 0x09,
		INVITATIONRIGHT   = 0x0A,
		SURPRISEBEHAVIOUR = 0x0B,
		BENDBACK          = 0x0C,
		BENDFORWARD       = 0x0D,
		BENDLEFT          = 0x0E,
		BENDRIGHT         = 0x0F,
		SETLOWSPEED       = 0x10,
		SETNORMALSPEED    = 0x11,
		SETHIGHSPEED      = 0x12,
		SETLOWACC         = 0x13,
		SETNORMALACC      = 0x14,
		SETHIGHACC        = 0x15,

//		Development Subroutines
		LEDON                = 0x2A,
		LEDOFF               = 0x2B,
		SETLED               = 0x2C,
		BLINKFAST10SECONDS   = 0x2D,
		BLINKSLOW10SECONDS   = 0x2E,
		LEDBLINKFASTNSECONDS = 0x2F,
		LEDBLINKSLOWNSECONDS = 0x30,
	};

	void initializePololuCommunication();
	void destroyPololuCommunication();

	void runSubroutine              (const int);
	void runSubroutine              (const int, const int);
	void runNeckSubroutine          (const int);
	void runFaceEmotionalSubroutine (const int);
	void runFaceSpeakingSubroutine  (const int);

	void stopScriptAndGoHome();

private:
	void transformParameter (const int, char&, char&);

	void waitForControllerScriptToComplete();
	void invitationLeft();
	void invitationRight();
	void give_a_bow();
	void expressSurprise();
	void reachStraightPosition();
	void reachStraightNeckPosition();
	void setLowSpeed();
	void setNormalSpeed();
	void setHighSpeed();
	void setLowAcc();
	void setNormalAcc();
	void setHighAcc();
};
