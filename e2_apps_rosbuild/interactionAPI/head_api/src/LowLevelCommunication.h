#ifndef LowLevelCommunication_h
#define LowLevelCommunication_h

#include "stl.h"
#include <string>

/**
 * Type for the lists of data used as interface towards the actuators
 */
typedef map<string,int> IntDataListType;

/**
 * This class is the interface of communication between the high-level controller
 * and the low-level one.
 *
 * The LowLevelCommunication class is an abstract class (in fact it has several
 * pure virtual methods) that gives the necessary interface to communicate with
 * the low-level controlling device.
 * This interface must be differently implemented for each communication device 
 * that will be used (e.g., serial, usb, i2c, etc.). In particular, it is required
 * to implement the methods to open and close the communication with the device,
 * and the methods that write towards the device and read from it.
 *
 * @short	Interface between high-level and low-level controllers
 * @author	Marcello Restelli (restelli@elet.polimi.it)
 */

class LowLevelCommunication
{
 public:
  // LIFECYCLE

  /**
   * Default constructor
   */
  LowLevelCommunication();

  /**
   * Destructor
   */
  virtual ~LowLevelCommunication();

  // OPERATORS

  //Use compiler-generated copy constructor and assignment
  //LowLevelCommunication(const LowLevelCommunication& from);
  //LowLevelCommunication& operator=(const LowLevelCommunication& from);

  // OPERATIONS
  
  /**
   * This is a pure virtual method that must be used to set up the communication
   * channel.
   */
  virtual void InitCommunication() = 0;
  
  /**
   * This is a pure virtual method that must be used to close the communication
   * channel.
   */
  virtual void CloseCommunication() = 0;

  /**
   * This is a pure virtual method that communicates the data stored in 
   * mActuationValues to the low-level controller
   */
  virtual void WriteActuations() = 0;

  /**
   * This is a pure virtual method that receives the data sent by the low-level
   * controller and stores them in mEncoderValues
   */
  virtual void ReadEncoders() = 0;

  /**
   * List of values that must be sent to the low-level controller
   */
  IntDataListType mActuationValues;

  /**
   * List of values that are received from the low-level controller
   */
  IntDataListType mEncoderValues;

};

#endif
