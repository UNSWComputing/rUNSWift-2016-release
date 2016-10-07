/*
 * SharedLocalisationUpdateBundle.hpp
 *
 *  Created on: 26/04/2014
 *      Author: osushkov
 */

#pragma once

#include "LocalisationUtils.hpp"
#include "LocalisationDefs.hpp"
#include "types/boostSerializationEigenTypes.hpp"
#include <Eigen/Eigen>
#include <vector>
#include "types/SharedBallInfo.hpp"

#define MAX_SHARED_BALLS 1

struct SharedLocalisationUpdateBundle {
   SharedLocalisationUpdateBundle() :
      ballSeenFraction(0.0),
      isUpdateValid(false),
      sharedUpdateMean(),
      sharedUpdateCovariance(),
      haveVisionUpdates(false),
      haveBallUpdates(false),
      sharedDx(0.0f),
      sharedDy(0.0f),
      sharedDh(0.0f),
      sharedCovarianceDx(0.0f),
      sharedCovarianceDy(0.0f),
      sharedCovarianceDh(0.0f) {}
   
   SharedLocalisationUpdateBundle(
         const float ballSeenFraction,
         const Eigen::MatrixXd &sharedUpdateMeanIn,
         const Eigen::MatrixXd &sharedUpdateCovarianceIn,
         const bool haveVisionUpdates,
         const bool haveBallUpdates,
         const double sharedDx, 
         const double sharedDy, 
         const double sharedDh,
         const double sharedCovarianceDx, 
         const double sharedCovarianceDy,
         const double sharedCovarianceDh,
         const std::vector<SharedBallInfo> sharedBallInfo) :
            ballSeenFraction(ballSeenFraction),
            isUpdateValid(true),
            haveVisionUpdates(haveVisionUpdates),
            haveBallUpdates(haveBallUpdates),
            sharedDx(sharedDx),
            sharedDy(sharedDy),
            sharedDh(sharedDh),
            sharedCovarianceDx(sharedCovarianceDx),
            sharedCovarianceDy(sharedCovarianceDy),
            sharedCovarianceDh(sharedCovarianceDh) {

      for (unsigned i = 0; i < MAX_SHARED_BALLS && i < (unsigned)sharedBallInfo.size(); i++)
      {
         sharedBalls[i] = sharedBallInfo[i];
      }
      
      for (int i = 0; i < SHARED_DIM; i++) {
         this->sharedUpdateMean(i, 0) = sharedUpdateMeanIn(i, 0);
      }
      
      for (int row = 0; row < SHARED_DIM; row++) {
         for (int col = 0; col < SHARED_DIM; col++) {
            this->sharedUpdateCovariance(row, col) = sharedUpdateCovarianceIn(row, col);
         }
      }
   }
   
   float ballSeenFraction;
   bool isUpdateValid;
   
   Eigen::Matrix<float, SHARED_DIM, 1> sharedUpdateMean;
   Eigen::Matrix<float, SHARED_DIM, SHARED_DIM> sharedUpdateCovariance;
   bool haveVisionUpdates;
   bool haveBallUpdates;
   
   float sharedDx;
   float sharedDy;
   float sharedDh;
   
   float sharedCovarianceDx;
   float sharedCovarianceDy;
   float sharedCovarianceDh;

   SharedBallInfo sharedBalls[MAX_SHARED_BALLS];
   
   void output(void) const {
      std::cout << "dx,dy,dz: " << sharedDx << " " << sharedDy << " " << sharedDh << std::endl;
      std::cout << "cdx,cdy,cdz: " << sharedCovarianceDx << " " << sharedCovarianceDy << " " 
            << sharedCovarianceDh << std::endl;
      
      prettyOutputMatrix("sharedUpdateMean", copyStaticToDynamicMatrix<SHARED_DIM, 1>(sharedUpdateMean));
      prettyOutputMatrix("sharedUpdateCovariance", copyStaticToDynamicMatrix<SHARED_DIM, SHARED_DIM>(sharedUpdateCovariance));
   }
   
   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & ballSeenFraction;
      ar & isUpdateValid;
      ar & sharedUpdateMean;
      ar & sharedUpdateCovariance;
      ar & haveVisionUpdates;
      ar & haveBallUpdates;
      ar & sharedDx;
      ar & sharedDy;
      ar & sharedDh;
      ar & sharedCovarianceDx;
      ar & sharedCovarianceDy;
      ar & sharedCovarianceDh;
      ar & sharedBalls;
   }
};

inline std::ostream& operator<<(std::ostream& os, const SharedLocalisationUpdateBundle& bundle) {
   os << (int) bundle.isUpdateValid << std::endl;
   
   for (int i = 0; i < SHARED_DIM; i++) {
      os << bundle.sharedUpdateMean(i, 0) << std::endl;
   }
   
   for (int i = 0; i < SHARED_DIM; i++) {
      for (int j = 0; j < SHARED_DIM; j++) {
         os << bundle.sharedUpdateCovariance(i, j) << std::endl;
      }
   }
   
   os << bundle.sharedDx << std::endl;
   os << bundle.sharedDy << std::endl;
   os << bundle.sharedDh << std::endl;
   
   os << bundle.sharedCovarianceDx << std::endl;
   os << bundle.sharedCovarianceDy << std::endl;
   os << bundle.sharedCovarianceDh << std::endl;

   for (unsigned i=0; i<MAX_SHARED_BALLS; i++)
   {
      os << bundle.sharedBalls[i];
   }
   
   return os;
}

inline std::istream& operator>>(std::istream& is, SharedLocalisationUpdateBundle& bundle) {
   int isUpdateValid;
   is >> isUpdateValid;
   bundle.isUpdateValid = (bool) isUpdateValid;
   
   for (int i = 0; i < SHARED_DIM; i++) {
      is >> bundle.sharedUpdateMean(i, 0);
   }
   
   for (int i = 0; i < SHARED_DIM; i++) {
      for (int j = 0; j < SHARED_DIM; j++) {
         is >> bundle.sharedUpdateCovariance(i, j);
      }
   }
   
   is >> bundle.sharedDx;
   is >> bundle.sharedDy;
   is >> bundle.sharedDh;
   
   is >> bundle.sharedCovarianceDx;
   is >> bundle.sharedCovarianceDy;
   is >> bundle.sharedCovarianceDh;

   for (unsigned i=0; i<MAX_SHARED_BALLS; i++)
   {
      SharedBallInfo b;
      is >> b;
      bundle.sharedBalls[i] = b;
   }
   
   return is;
}
