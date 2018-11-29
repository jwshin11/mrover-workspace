#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
  Searcher(StateMachine* stateMachine_);

  virtual ~Searcher() {}

  NavState run();

  // Queue of search points.
  queue<Odometry> mSearchPoints;

  // Number of waypoints missed.
  unsigned mMissedWaypoints = 0;

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

  virtual void initializeSearch() = 0;

  virtual bool addFourPointsToSearch() = 0;

  // Vector of search point multipliers used as a base for the search
  // points.

protected:
  vector< pair<short, short> > mSearchPointMultipliers;


public:

    SearchState currentState;

    StateMachine* stateMachine;

};

#endif //SEARCHER_HPP
