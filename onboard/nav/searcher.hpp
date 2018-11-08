#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class Searcher {
public:
  Searcher();
  NavState run();
  // Queue of search points.
  queue<Odometry> mSearchPoints;
  void UpdateRover(Rover* rover);

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

  void initializeSearch();

  bool addFourPointsToSearch();


  // Vector of search point multipliers used as a base for the search
  // points.
  vector< pair<short, short> > mSearchPointMultipliers;

  SearchState currentState;

  Rover * mPhoebe;
};

#endif //SEARCHER_HPP
