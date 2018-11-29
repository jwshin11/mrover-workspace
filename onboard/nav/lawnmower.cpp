#ifndef LAWNMOWER_HPP
#define LAWNMOWER_HPP

#include "searcher.hpp"
#include "rover.hpp"

//class StateMachine;

class Lawnmower : public Searcher {

  // Initializes the search point multipliers to be the intermost loop
  // of the search.
  void initializeSearch()
  {
    clear( mSearchPoints );
  	mSearchPointMultipliers.clear();
    mSearchPointMultipliers.push_back( pair<short, short> ( 0 , 0) );
  	mSearchPointMultipliers.push_back( pair<short, short> ( 0 , 1 ));
  	mSearchPointMultipliers.push_back( pair<short, short> ( 1 , 1 ));
  	mSearchPointMultipliers.push_back( pair<short, short> ( 1 , 0) );
  	addFourPointsToSearch();
  }// initializeSearch()

  // true indicates to added search points
  // Add the next loop to the search. If the points are added to the
  // search, returns true. If the rover is further away from the start
  // of the search than the search bail threshold, return false.

  bool addFourPointsToSearch()
  {
    const double pathWidth = stateMachine->mRoverConfig[ "pathWidth" ].GetDouble();
    if( mSearchPointMultipliers[ 0 ].second * pathWidth > stateMachine->mRoverConfig[ "searchBailThresh" ].GetDouble() )
    {
      return false;
    }

    for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
    {
      Odometry nextSearchPoint = stateMachine->mPhoebe->roverStatus().path().front().odom;
      double totalLatitudeMinutes = nextSearchPoint.latitude_min +
        ( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
      double totalLongitudeMinutes = nextSearchPoint.longitude_min +
      ( mSearchPointMultiplier.second * (SW - 2*cvThresh) * stateMachine->mPhoebe->longMeterInMinutes() );

      nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
      nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
      nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
      nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

      mSearchPoints.push( nextSearchPoint );

		++++mSearchPointMultiplier.first;
    }
    return true;
  }// addFourPointsToSearch()

};

#endif //LAWNMOWER_HPP
