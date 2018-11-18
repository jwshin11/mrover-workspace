#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
  Searcher(StateMachine* stateMachine_);
  
  NavState run();
  
  Odometry frontSearchPoint();

  void popSearchPoint();

private:
  enum class SearchState
  {
    SearchFaceNorth,
    SearchTurn120,
    SearchTurn240,
    SearchTurn360,
    SearchTurn,
    SearchDrive,
    TurnToBall,
    DriveToBall
  };

  NavState executeSearchFaceNorth();

  NavState executeSearchTurn120();

  NavState executeSearchTurn240();

  NavState executeSearchTurn360();

  NavState executeSearchTurn();

  NavState executeSearchDrive();

  NavState executeTurnToBall();

  NavState executeDriveToBall();

  void initializeSearch();

  bool addFourPointsToSearch();

  // Vector of search point multipliers used as a base for the search
  // points.
  vector< pair<short, short> > mSearchPointMultipliers;


public:
    
    SearchState currentState;

    StateMachine* stateMachine;

    // Queue of search points.
    queue<Odometry> mSearchPoints;

    // Number of waypoints missed.
    unsigned mMissedWaypoints = 0;

};

#endif //SEARCHER_HPP
