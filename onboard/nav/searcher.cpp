Searcher::Searcher()
  :currentState(SearchFaceNorth)  { }

Navstate Searcher::run()
{
  switch (currentState)
    {
    case State::SearchFaceNorth:
    {
      return executeSearchFaceNorth();

    }

    case State::SearchTurn120:
    {
      return executeSearchTurn120();

    }

    case State::SearchTurn240:
    {
      return executeSearchTurn240();

    }

    case State::SearchTurn360:
    {
      return executeSearchTurn360();

    }

    case State::SearchTurn:
    {
      return executeSearchTurn();

    }

    case State::SearchDrive:
    {
      return executeSearchDrive();

    }

    case State::TurnToBall:
    {
      return executeTurnToBall();

    }

    case State::DriveToBall:
    {
      return executeDriveToBall();

    }
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
  currentState = SearchState::Turn;
  return NavState::Search;
} // executeSearchTurn()

// Executes the logic for driving while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes driving,
// it proceeds to turning to the next Waypoint. If the rover detects
// an obstacle, it proceeds to obstacle avoidance. Else the rover
// keeps driving to the next Waypoint.
State Searcher::executeSearchDrive()
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
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
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
State Searcher::executeTurnToBall()
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
State Searcher::executeDriveToBall()
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
		mOriginalObstacleAngle = mPhoebe->roverStatus().obstacle().bearing;
    currentState = SearchState::SearchFaceNorth;
		return NavState::SearchTurnAroundObs;
	}
	DriveStatus driveStatus = mPhoebe->drive( mPhoebe->roverStatus().tennisBall().distance,
											mPhoebe->roverStatus().tennisBall().bearing );
	if( driveStatus == DriveStatus::Arrived )
	{
		mPhoebe->roverStatus().path().pop();
		++mCompletedWaypoints;
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
