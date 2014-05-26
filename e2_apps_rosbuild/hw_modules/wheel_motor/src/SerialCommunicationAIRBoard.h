#ifndef SerialCommunicationAIRBoard_h
#define SerialCommunicationAIRBoard_h

#include "SerialCommunication.h"


/**
 * This class implements the communication thorugh serial device toward the
 * control board produced inside the AIRLab.
 *
 * The SerialCommunicationAIRBoard class implements the methods to write to and
 * read from the AIRBoard control board. In particular, this class implements
 * two pure virtual methods defined in the @ref LowLevelCommunication interface.
 *
 * @short	Interface for the AIRBoard control board
 * @author	Marcello Restelli (restelli@elet.polimi.it)
 */

class SerialCommunicationAIRBoard : public SerialCommunication
{
 public:
  // LIFECYCLE

  /**
   * Default constructor
   */
  SerialCommunicationAIRBoard();

  /**
   * Destructor
   */
  virtual ~SerialCommunicationAIRBoard();


  // OPERATORS

  //Use compiler-generated copy constructor and assignment
  //SerialCommunicationAIRBoard(const SerialCommunicationAIRBoard& from);
  //SerialCommunicationAIRBoard& operator=(const SerialCommunicationAIRBoard& from);

  // OPERATIONS
  
  /**
   * This is a pure virtual method that communicates the data stored in 
   * mActuationValues to the AIRBoard
   */
  void WriteActuations();

  /**
   * This is a pure virtual method that receives the data sent by the AIRBoard
   * and stores them in mEncoderValues
   */
  void ReadEncoders();

  /**
   * Set to zero the speed of the motors and  enables them
   */
  void StartMotors();

  /**
   * Set to zero the speed of the motors and  disables them
   */
  void StopMotors();

 protected:

  /**
   * Set the speeds for motor 0 and motor 1
   *
   * @param speed_motor_0	The speed of motor 0 (between -99 and +99)
   * @param speed_motor_1	The speed of motor 1 (between -99 and +99)
   */
  void SetMotorsSpeed(int speed_motor_0, int speed_motor_1);


  /**
   * Send the kick command
   *
   * @param kicker	The kicker that must be activated (1: front, -1:rear,
   *			2:both)			
   */
  void Kick(int kicker);

 private:
 
		int last_speed_motor_0;
		int last_speed_motor_1;


};

#endif
