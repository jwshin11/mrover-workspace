//#ifndef SPIRAL_HPP
//#define SPIRAL_HPP

//#include "searcher.hpp"
#include "searcherTypes.hpp"
#include "rover.hpp"
//#include "stateMachine.hpp"
#include "utilities.hpp"

  Spiral::Spiral(StateMachine* sm)
    : Searcher(sm) { }

  // Initializes the search point multipliers to be the intermost loop
  // of the search.
  void Spiral::initializeSearch()
  {
    clear( mSearchPoints );
  	mSearchPointMultipliers.clear();
  	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
  	mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
  	mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
  	mSearchPointMultipliers.push_back( pair<short, short> ( 1, -1 ) );
  	addFourPointsToSearch();
  }// initializeSearch()

  // true indicates to added search points
  // Add the next loop to the search. If the points are added to the
  // search, returns true. If the rover is further away from the start
  // of the search than the search bail threshold, return false.

  bool Spiral::addFourPointsToSearch()
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
        ( mSearchPointMultiplier.second * pathWidth * stateMachine->mPhoebe->longMeterInMinutes() );

      nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
      nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
      nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
      nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

      mSearchPoints.push( nextSearchPoint );

      mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
      mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
    }
    return true;
  }// addFourPointsToSearch()

//#endif //SPIRAL_HPP
