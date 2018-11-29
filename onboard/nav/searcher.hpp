#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
  /*************************************************************************/
  /* Public Member Functions */
  /*************************************************************************/
  Searcher(StateMachine* stateMachine_)
  : currentState(SearchState::SearchFaceNorth)
  , stateMachine(stateMachine_)
  , search_fails(0) { }

  virtual ~Searcher() { }

  NavState run();

  Odometry frontSearchPoint();

  void popSearchPoint();

private:
  /*************************************************************************/
  /* Private Enum Class */
  /*************************************************************************/
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

  /*************************************************************************/
  /* Private Member Functions */
  /*************************************************************************/
  NavState executeSearchFaceNorth();

  NavState executeSearchTurn120();

  NavState executeSearchTurn240();

  NavState executeSearchTurn360();

  NavState executeSearchTurn();

  NavState executeSearchDrive();

  NavState executeTurnToBall();

  NavState executeDriveToBall();

  virtual void initializeSearch() = 0;

  virtual bool addPointsToSearch() = 0;

protected:
  /*************************************************************************/
  /* Protected Member Variables */
  /*************************************************************************/

  SearchState currentState;

  StateMachine* stateMachine;

  int search_fails;

  // Vector of search point multipliers used as a base for the search points.
  vector< pair<short, short> > mSearchPointMultipliers;

  // Queue of search points.
  queue<Odometry> mSearchPoints;

};

#endif //SEARCHER_HPP
