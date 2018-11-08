#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "rover.hpp"
#include "searcher.hpp"

#include <lcm/lcm-cpp.hpp>
#include <queue>
#include "rapidjson/document.h"

using namespace std;
using namespace rover_msgs;

// This class implements the logic for the state machine for the
// autonomous navigation of the rover.
class StateMachine
{
public:
	/*************************************************************************/
	/* Public Member Functions */
	/*************************************************************************/
	StateMachine( lcm::LCM& lcmObject );

	~StateMachine();

	void run();

	void updateRoverStatus( AutonState autonState );

	void updateRoverStatus( Course course );

	void updateRoverStatus( Obstacle obstacle );

	void updateRoverStatus( Odometry odometry );

	void updateRoverStatus( TennisBall tennisBall );

private:
	/*************************************************************************/
	/* Private Member Functions */
	/*************************************************************************/
	void publishNavState() const; // todo check if lcm const member

	NavState executeOff();

	NavState executeDone();

	NavState executeTurn();

	NavState executeDrive();

	NavState executeSearch();

	NavState executeTurnAroundObs();

	NavState executeDriveAroundObs();

	void initializeSearch();

	bool addFourPointsToSearch();

	Odometry createAvoidancePoint( const double distance );

	/*************************************************************************/
	/* Private Member Variables */
	/*************************************************************************/
	// Rover object to do basic rover operations in the state machine.
	Rover* mPhoebe;

	// RoverStatus object for updating the rover's status.
	Rover::RoverStatus mNewRoverStatus;

	// Lcm object for sending and recieving messages.
	lcm::LCM& mLcmObject;

	// Configuration file for the rover.
	rapidjson::Document mRoverConfig;

	// Odometry point used when avoiding obstacles.
	Odometry mObstacleAvoidancePoint;

	// Initial angle to go around obstacle upon detection.
	double mOriginalObstacleAngle;

	// Number of waypoints in course.
	unsigned mTotalWaypoints;

	// Number of waypoints completed.
	unsigned mCompletedWaypoints;

	// Indicates if the state changed on a given iteration of run.
	bool mStateChanged;

	// Search object to control all search states
	// TODO better way to pass pointer?
	// Searcher searcher(mPhoebe);

}; // StateMachine

#endif // STATE_MACHINE_HPP
