#include "searcher.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>

Odometry Searcher::frontSearchPoint()
{
	return mSearchPoints.front();
}

void Searcher::popSearchPoint()
{
	mSearchPoints.pop();
	return;
}

NavState Searcher::run( Rover * mPhoebe, const rapidjson::Document& mRoverConfig )
{
	switch ( mPhoebe->roverStatus().currentState() )
	{
	    case NavState::SearchFaceNorth:
	    {
	      return executeSearchFaceNorth( mPhoebe );
	    }

	    case NavState::SearchFace120:
	    {
	      return executeSearchFace120( mPhoebe );
	    }

	    case NavState::SearchFace240:
	    {
	      return executeSearchFace240( mPhoebe );
	    }

	    case NavState::SearchFace360:
	    {
	      return executeSearchFace360( mPhoebe, mRoverConfig );
	    }

	    case NavState::SearchTurn:
	    {
	      return executeSearchTurn( mPhoebe, mRoverConfig );
	    }

	    case NavState::SearchDrive:
	    {
	      return executeSearchDrive( mPhoebe );
	    }

	    case NavState::TurnToBall:
	    {
	      return executeTurnToBall( mPhoebe );
	    }

	    case NavState::DriveToBall:
	    {
	      return executeDriveToBall( mPhoebe );
	    }
        default:
        {
            return NavState::Unknown;
        }
	} // switch
}

// Executes the logic for turning to face north to orient itself for
// a search. If the rover is turned off, it proceeds to Off. If the
// rover detects the tennis ball, it proceeds to the ball If the rover
// finishes turning, it proceeds to SearchFace120. Else the rover keeps
// turning to north.
NavState Searcher::executeSearchFaceNorth( Rover * mPhoebe )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 90 ) )
	{
		return NavState::SearchFace120;
	}
	return NavState::SearchFaceNorth;
} // executeSearchFaceNorth

// Executes the logic for the first third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchFace240.
// Else the rover keeps turning to 120 degrees.
NavState Searcher::executeSearchFace120( Rover * mPhoebe )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 210 ) )
	{
		return NavState::SearchFace240;
	}

    return NavState::SearchFace120;
} // executeSearchFace120()

// Executes the logic for the second third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchFace360.
// Else the rover keeps turning to 240 degrees.
NavState Searcher::executeSearchFace240( Rover * mPhoebe )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    	return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 330 ) )
	{
    	return NavState::SearchFace360;
	}
  	return NavState::SearchFace240;
} // executeSearchFace240

// Executes the logic for the final third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, the next state is SearchDrive.
// Else the rover keeps turning to 360 degrees.
NavState Searcher::executeSearchFace360( Rover* mPhoebe, const rapidjson::Document& mRoverConfig )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    	return NavState::TurnToBall;
	}
	if( mPhoebe->turn( 90 ) )
	{
    	return NavState::SearchTurn;
	}
  return NavState::SearchFace360;
} // executeSearchFace360()

// Executes the logic for turning while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes turning,
// it proceeds to driving while searching. Else the rover keeps
// turning to the next Waypoint.
NavState Searcher::executeSearchTurn( Rover* mPhoebe, const rapidjson::Document& mRoverConfig )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    	return NavState::TurnToBall;
	}

	if( mSearchPoints.empty() )
	{
		static int searchFails = 0;

		if ( mRoverConfig["searchOrderSize"].GetInt() <= searchFails) {
			mPhoebe->roverStatus().path().pop();
			stateMachine->updateMissedPoints();
			return NavState::Turn;
		}
		else {
			switch(mRoverConfig[ "searchOrder" ][ searchFails ].GetInt())
			{
				case 0:
					stateMachine->setSearcher(SearchType::SPIRALOUT);
					break;
				case 1:
					stateMachine->setSearcher(SearchType::LAWNMOWER);
					break;
				case 2:
					stateMachine->setSearcher(SearchType::SPIRALIN);
					break;
				default:
					stateMachine->setSearcher(SearchType::SPIRALOUT);
					break;
			}
			initializeSearch( mPhoebe, mRoverConfig );
			searchFails += 1;
		}
	}

	Odometry& nextSearchPoint = mSearchPoints.front();
	if( mPhoebe->turn( nextSearchPoint ) )
	{
   		return NavState::SearchDrive;
	}
  return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes driving,
// it proceeds to turning to the next Waypoint. If the rover detects
// an obstacle, it proceeds to obstacle avoidance. Else the rover
// keeps driving to the next Waypoint.
NavState Searcher::executeSearchDrive( Rover * mPhoebe )
{
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    	return NavState::TurnToBall;
	}

	if( mPhoebe->roverStatus().obstacle().detected )
	{
        stateMachine->updateObstacleAngle(mPhoebe->roverStatus().obstacle().bearing);
    	// currentState = SearchState::SearchTurn; // todo
		return NavState::SearchTurnAroundObs;
	}

	const Odometry& nextSearchPoint = mSearchPoints.front();
	DriveStatus driveStatus = mPhoebe->drive( nextSearchPoint );
	if( driveStatus == DriveStatus::Arrived )
	{
		mSearchPoints.pop();
		return NavState::SearchTurn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return NavState::SearchDrive;
	}
  	return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts to search again. If the rover finishes turning to the ball,
// it drives to the ball. Else the rover continues to turn to to the
// ball.
NavState Searcher::executeTurnToBall( Rover * mPhoebe )
{
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::SearchFaceNorth;
	}
	if( mPhoebe->turn( mPhoebe->roverStatus().tennisBall().bearing +
					mPhoebe->roverStatus().bearing().bearing ) )
	{
		return NavState::DriveToBall;
	}
    return NavState::TurnToBall;
} // executeTurnToBall()

// Executes the logic for driving to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts the search again. If the rover detects an obstacle, it
// proceeds to go around the obstacle. If the rover finishes driving
// to the ball, it moves on to the next Waypoint. If the rover gets
// off course, it proceeds to turn back to the Waypoint. Else, it
// continues driving to the ball.
NavState Searcher::executeDriveToBall( Rover * mPhoebe )
{
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
		return NavState::SearchFaceNorth;
	}
	// TODO: save location of ball then go around object?
	if( mPhoebe->roverStatus().obstacle().detected )
	{
		stateMachine->updateObstacleAngle(mPhoebe->roverStatus().obstacle().bearing);
		// currentState = SearchState::SearchFaceNorth; // todo
		return NavState::SearchTurnAroundObs;
	}
	DriveStatus driveStatus = mPhoebe->drive( mPhoebe->roverStatus().tennisBall().distance,
											mPhoebe->roverStatus().tennisBall().bearing +
                                            mPhoebe->roverStatus().bearing().bearing );
	if( driveStatus == DriveStatus::Arrived )
	{
		mPhoebe->roverStatus().path().pop();
        stateMachine->updateCompletedPoints();
    	// currentState = SearchState::SearchFaceNorth; // todo
		return NavState::Turn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
		return NavState::DriveToBall;
	}
  	return NavState::TurnToBall;
} // executeDriveToBall()
