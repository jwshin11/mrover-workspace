#ifndef SEARCHES_HPP
#define SEARCHES_HPP

#include "searcher.hpp"

enum class SearchType
{
	SPIRAL,
	LAWNMOWER,
	UNKNOWN
};

Searcher* SearchFactory(StateMachine* stateMachine, SearchType type);

/*************************************************************************/
/* Spiral Search */
/*************************************************************************/
class Spiral : public Searcher 
{
public:
	Spiral(StateMachine* stateMachine_) 
	: Searcher(stateMachine_) {}

	~Spiral();

	// Initializes the search ponit multipliers to be the intermost loop
 	// of the search.
	void initializeSearch();

	// true indicates to added search points
	// Add the next loop to the search. If the points are added to the
	// search, returns true. If the rover is further away from the start
	// of the search than the search bail threshold, return false.
	bool addFourPointsToSearch();
};

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public Searcher 
{
public:
	LawnMower(StateMachine* stateMachine_) 
	: Searcher(stateMachine_) {}

	~LawnMower();

	// Initializes the search ponit multipliers to be the intermost loop
 	// of the search.
	void initializeSearch();

	// true indicates to added search points
	// Add the next loop to the search. If the points are added to the
	// search, returns true. If the rover is further away from the start
	// of the search than the search bail threshold, return false.
	bool addFourPointsToSearch();
};

#endif //SEARCHES_HPP