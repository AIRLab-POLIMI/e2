#ifndef GianoActuationManager_h
#define GianoActuationManager_h

#include "ActuationManager.h"

/**
 * This class implements the translation of actions and odometry for robots
 * with the Giano kinematics (i.e., two parallel wheels that are fixed to the
 * chassis).
 *
 * The GianoActuationManager class implements the method for translating the
 * actions received by the high-level controller and the method for computing
 * the robot movement starting from the odometric data acquired by the low-level
 * control board, according to the Giano kinematics. In particular, the
 * high-level controller will produce commands for the tangential speed and for
 * the rotational speed that must be translated in commands for the motors. On
 * the other hand, from the readings of the encoders this class will produce the
 * actual movement of the robot.
 *
 * @short	Implementation of the ActuationManager for the Giano kinematics
 * @author	Marcello Restelli (restelli@elet.polimi.it)
 */

class GianoActuationManager : public ActuationManager
{
 public:
  // LIFECYCLE

  /**
   * Default constructor
   */
  GianoActuationManager();

  /**
   * Destructor
   */
  virtual ~GianoActuationManager();


  // OPERATORS

  //Use compiler-generated copy constructor and assignment
  //GianoActuationManager(const GianoActuationManager& from);
  //GianoActuationManager& operator=(const GianoActuationManager& from);

  // OPERATIONS

  /**
   * Decompose the actions received by the high-level controller
   * into commands for the low-level controller
   */ 
  void ComputeActuations(float tanSpeed, float rotSpeed);

  /**
   * Starting from the data of the encoders, this method computes the robot
   * movement
   */
  void ComputeOdometry();
  
  /**
   * Set the distance between the robot's wheels
   */
  void SetWheelDistance( float wheeldistance );

  /**
   * Set the coefficient used to convert from tics to mm
   */
  void SetConversionCoefficients( float coefficient_sx, float coefficient_dx );

 private:
  float mWheelDistance;
  float mCoefficientTicTomm_Sx;   // coefficient used to convert from wheels' tics to millimeters
  float mCoefficientTicTomm_Dx;
};

#endif
