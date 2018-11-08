#include "searcher.hpp"
#include "utilities.hpp"

Searcher::Searcher()
  : currentState(SearchState::SearchFaceNorth) { }

void Searcher::UpdateRover(Rover * rover)
{
    mPhoebe = rover;
}
NavState Searcher::run()
{
  switch (currentState)
    {
    case SearchState::SearchFaceNorth:
    {
      return executeSearchFaceNorth();

    }

    case SearchState::SearchTurn120:
    {
      return executeSearchTurn120();

    }

    case SearchState::SearchTurn240:
    {
      return executeSearchTurn240();

    }

    case SearchState::SearchTurn360:
    {
      return executeSearchTurn360();

    }

    case SearchState::SearchTurn:
    {
      return executeSearchTurn();

    }

    case SearchState::SearchDrive:
    {
      return executeSearchDrive();

    }

    case SearchState::TurnToBall:
    {
      return executeTurnToBall();

    }

    case SearchState::DriveToBall:
    {
      return executeDriveToBall();
    }

    return NavState::Unknown;
  }
}

// Executes the logic for turning to face north to orient itself for
// a search. If the rover is turned off, it proceeds to Off. If the
// rover detects the tennis ball, it proceeds to the ball If the rover
// finishes turning, it proceeds to SearchTurn120. Else the rover keeps
// turning to north.
NavState Searcher::executeSearchFaceNorth()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
		return NavState::Search;
	}
	if( mPhoebe->turn( 90 ) )
	{
    currentState = SearchState::SearchTurn120;
		return NavState::Search;
	}
  currentState = SearchState::SearchFaceNorth;
	return NavState::Search;
} // executeSearchFaceNorth

// Executes the logic for the first third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchTurn240.
// Else the rover keeps turning to 120 degrees.
NavState Searcher::executeSearchTurn120()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
		return NavState::Search;
	}
	if( mPhoebe->turn( 210 ) )
	{
    currentState = SearchState::SearchTurn240;
		return NavState::Search;
	}
  currentState = SearchState::SearchTurn120;
  return NavState::Search;
} // executeSearchTurn120()

// Executes the logic for the second third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchTurn360.
// Else the rover keeps turning to 240 degrees.
NavState Searcher::executeSearchTurn240()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
    return NavState::Search;
	}
	if( mPhoebe->turn( 330 ) )
	{
    currentState = SearchState::SearchTurn360;
    return NavState::Search;
	}
  currentState = SearchState::SearchTurn240;
  return NavState::Search;
} // executeSearchTurn240

// Executes the logic for the final third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, the next state is SearchDrive.
// Else the rover keeps turning to 360 degrees.
NavState Searcher::executeSearchTurn360()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
    return NavState::Search;
	}
	if( mPhoebe->turn( 90 ) )
	{
		initializeSearch();
    currentState = SearchState::SearchTurn;
    return NavState::Search;
	}
  currentState = SearchState::SearchTurn360;
  return NavState::Search;
} // executeSearchTurn360()

// Executes the logic for turning while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes turning,
// it proceeds to driving while searching. Else the rover keeps
// turning to the next Waypoint.
NavState Searcher::executeSearchTurn()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
    return NavState::Search;
	}
	if( mSearchPoints.empty() )
	{
		if( !addFourPointsToSearch() )
		{
			mPhoebe->roverStatus().path().pop();
			++mMissedWaypoints;
      currentState = SearchState::SearchFaceNorth;
			return NavState::Turn;
		}
		// return State::SearchTurn;
	}

	Odometry& nextSearchPoint = mSearchPoints.front();
	if( mPhoebe->turn( nextSearchPoint ) )
	{
    currentState = SearchState::SearchDrive;
    return NavState::Search;
	}
  currentState = SearchState::SearchTurn;
  return NavState::Search;
} // executeSearchTurn()

// Executes the logic for driving while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes driving,
// it proceeds to turning to the next Waypoint. If the rover detects
// an obstacle, it proceeds to obstacle avoidance. Else the rover
// keeps driving to the next Waypoint.
NavState Searcher::executeSearchDrive()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::TurnToBall;
    return NavState::Search;
	}
	if( mPhoebe->roverStatus().obstacle().detected )
	{
        // TODO find way to return obstacle bearing
        // mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
    currentState = SearchState::SearchTurn;
		return NavState::SearchTurnAroundObs;
	}

	const Odometry& nextSearchPoint = mSearchPoints.front();
	DriveStatus driveStatus = mPhoebe->drive( nextSearchPoint );
	if( driveStatus == DriveStatus::Arrived )
	{
		mSearchPoints.pop();
    currentState = SearchState::SearchTurn;
		return NavState::Search;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
    currentState = SearchState::SearchDrive;
		return NavState::Search;
	}
	// if driveStatus == DriveStatus::OffCourse
  currentState = SearchState::SearchTurn;
  return NavState::Search;
} // executeSearchDrive()

// Executes the logic for turning to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts to search again. If the rover finishes turning to the ball,
// it drives to the ball. Else the rover continues to turn to to the
// ball.
NavState Searcher::executeTurnToBall()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Search;
	}
	if( mPhoebe->turn( mPhoebe->roverStatus().tennisBall().bearing ) )
	{
    currentState = SearchState::DriveToBall;
		return NavState::Search;
	}
  currentState = SearchState::TurnToBall;
  return NavState::Search;
} // executeTurnToBall()

// Executes the logic for driving to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts the search again. If the rover detects an obstacle, it
// proceeds to go around the obstacle. If the rover finishes driving
// to the ball, it moves on to the next Waypoint. If the rover gets
// off course, it proceeds to turn back to the Waypoint. Else, it
// continues driving to the ball.
NavState Searcher::executeDriveToBall()
{
	if( !mPhoebe->roverStatus().autonState().is_auton )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Off;
	}
	if( !mPhoebe->roverStatus().tennisBall().found )
	{
    currentState = SearchState::SearchFaceNorth;
		return NavState::Search;
	}
	// TODO: save location of ball then go around object?
	if( mPhoebe->roverStatus().obstacle().detected )
	{
        // TODO find way to return obstacle bearing
        //mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
    currentState = SearchState::SearchFaceNorth;
		return NavState::SearchTurnAroundObs;
	}
	DriveStatus driveStatus = mPhoebe->drive( mPhoebe->roverStatus().tennisBall().distance,
											mPhoebe->roverStatus().tennisBall().bearing );
	if( driveStatus == DriveStatus::Arrived )
	{
		mPhoebe->roverStatus().path().pop();
        // TODO find way to increment waypont
        //++mCompletedWaypoints;
    currentState = SearchState::SearchFaceNorth;
		return NavState::Turn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
    currentState = SearchState::DriveToBall;
		return NavState::Search;
	}
	// if driveStatus == DriveStatus::OffCourse
  currentState = SearchState::TurnToBall;
  return NavState::Search;
} // executeDriveToBall()

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void Searcher::initializeSearch()
{
	clear( mSearchPoints );
	mSearchPointMultipliers.clear();
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, -1 ) );
	addFourPointsToSearch();
} // initializeSearch()

// true indicates to added search points

// Add the next loop to the search. If the points are added to the
// search, returns true. If the rover is further away from the start
// of the search than the search bail threshold, return false.
bool Searcher::addFourPointsToSearch()
{
	const double pathWidth = mRoverConfig[ "pathWidth" ].GetDouble();
	if( mSearchPointMultipliers[ 0 ].second * pathWidth > mRoverConfig[ "searchBailThresh" ].GetDouble() )
	{
		return false;
	}

	for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
	{
		Odometry nextSearchPoint = mPhoebe->roverStatus().path().front().odom;
		double totalLatitudeMinutes = nextSearchPoint.latitude_min +
			( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
		double totalLongitudeMinutes = nextSearchPoint.longitude_min +
			( mSearchPointMultiplier.second * pathWidth * mPhoebe->longMeterInMinutes() );

		nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
		// nextSearchPoint.latitude_min += mod( totalLatitudeMinutes, 60 );
		// printf("%f\n", nextSearchPoint.latitude_min);
		nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
		// printf("%f\n", nextSearchPoint.latitude_min);
		nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
		// nextSearchPoint.longitude_min += mod( totalLongitudeMinutes, 60 );
		nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

		mSearchPoints.push( nextSearchPoint );

		mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
		mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
	}
	return true;
} // addFourPointsToSearch()
