#pragma once

#include "types/BallInfo.hpp"
#include "types/RRCoord.hpp"
#include "types/AbsCoord.hpp"
#include "perception/localisation/ICP.hpp"



/* Converts RRCoord to AbsCoord */
//extern static AbsCoord rrToAbs(const RRCoord obs, const AbsCoord robotPos);

/* BallInfo is too big to send over broadcast so we're making little BallInfo babies to send instead */
struct SharedBallInfo
{
   	SharedBallInfo ()
   	{
   		visionVar = -1;
   	}

   	SharedBallInfo (const BallInfo& ball, const AbsCoord robotPos)
   	{
   		visionVar = ball.visionVar;
   		lastSeen = ball.lastSeen;
   		lifetime = ball.lifetime;
   		// TODO fix this
   		globalPosition = rrToAbs(ball.rr, robotPos);
   	}

   	/* A measure of truth as provided by vision */
  	float visionVar;

 	/* The estimated global position of the ball */
   	AbsCoord globalPosition;

   	/* Number of frames since this ball estimate was seen */
   	int lastSeen;

   	/* Length of life for this ball so far */
   	int lifetime;
  
  bool operator== (const SharedBallInfo &other) const
   {
      return globalPosition == other.globalPosition
          && visionVar    == other.visionVar
          && lastSeen     == other.lastSeen
          && lifetime     == other.lifetime;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
   		ar & globalPosition;
      	ar & visionVar;
      	ar & lastSeen;
      	ar & lifetime;
   }

   BallInfo toBallInfo(AbsCoord robotPos)
   {
   		BallInfo b;
	    b.visionVar = visionVar;
	    b.lastSeen = lastSeen;
	    b.lifetime = lifetime;
	    b.rr = globalPosition.convertToRobotRelative(robotPos);
	    return b;
   }
};


inline std::ostream& operator<<(std::ostream& os, const SharedBallInfo& sharedBallInfo) {

   os << sharedBallInfo.globalPosition;
   os.write((char*) &(sharedBallInfo.visionVar), sizeof(float));
   os.write((char*) &(sharedBallInfo.lastSeen), sizeof(int));
   os.write((char*) &(sharedBallInfo.lifetime), sizeof(int));
   return os;
}

inline std::istream& operator>>(std::istream& is, SharedBallInfo& sharedBallInfo) {

   is >> sharedBallInfo.globalPosition;
   is.read((char*) &(sharedBallInfo.visionVar), sizeof(float));
   is.read((char*) &(sharedBallInfo.lastSeen), sizeof(int));
   is.read((char*) &(sharedBallInfo.lifetime), sizeof(int));
   return is;
}