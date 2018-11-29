#ifndef SEARCHERTYPES_HPP
#define SEARCHERTYPES_HPP

#include "searcher.hpp"
//#include "rover.hpp"
#include "stateMachine.hpp"
//#include "utilities.hpp"

enum searcher_type {
  Spiral_type,
  Lawnmower_type
};

class Spiral : public Searcher {
public:
  Spiral(StateMachine* sm);

private:
  void initializeSearch();

  bool addFourPointsToSearch();

};

class Lawnmower : public Searcher {
public:
  Lawnmower(StateMachine* sm);

private:
  void initializeSearch();

  bool addFourPointsToSearch();

};

#endif //SEARCHER_TYPES_HPP
