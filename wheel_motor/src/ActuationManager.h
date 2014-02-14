
#ifndef ActuationManager_h
#define ActuationManager_h


#include "SerialCommunicationAIRBoard.h"
#include <string>
#include <iostream>
#include <map>

typedef std::map <std::string,float> FloatDataListType;

/**
 * This class is the interface for the data sent and received by the high-level
 * controller. 
 *
 * The ActuationManager class is an abstract class (in fact it has two
 * pure virtual methods) that gives the necessary interface to translate
 * the data between the high-level controller and the low-level controller
 * according to the different kinematics (e.g.: holonomous, anholonomous, 
 * synchro, etc.).
 *
 * @short	Interface between high-level and low-level control data
 * @author	Marcello Restelli (restelli@elet.polimi.it)
 */

class ActuationManager: public SerialCommunicationAIRBoard
{
 public:
  // LIFECYCLE

  /**
   * Default constructor
   */
  ActuationManager();

  /**
   * Destructor
   */
  virtual ~ActuationManager();

  // OPERATORS

  //Use compiler-generated copy constructor and assignment
  //ActuationManager(const ActuationManager& from);
  //ActuationManager& operator=(const ActuationManager& from);

  // OPERATIONS

  /**
   * Insert the data from the high-controller into a list
   *
   * @param name	The symbolic name of the datum
   * @param value	The value of the datum
   */
  void InsertData(std::string name, float value);

  /**
   * Remove the data from the list
   *
   * @param name	The symbolic name of the datum
   */
  void RemoveData(std::string name);

  /**
   * Decompose the actions received by the high-level controller
   * into commands for the low-level controller
   */ 
  virtual void ComputeActuations(float tanSpeed, float rotSpeed) = 0;

  /**
   * Starting from the data of the encoders, this method computes the robot
   * movement
   */
  virtual void ComputeOdometry() = 0;

 protected:
  /**
   * The list of data received from the high-level controller
   */
  FloatDataListType mCommandData;

  /**
   * The list of data that must be sent to the high-level controller
   */
  FloatDataListType mOdometryData;
};


inline void ActuationManager::InsertData(std::string name, float value)
{
  mCommandData[name] = value;
}

inline void ActuationManager::RemoveData(std::string name)
{
  FloatDataListType::iterator it = mCommandData.find(name);
  if (it != mCommandData.end())
  {
    mCommandData.erase(it);
  }
}

#endif
