#ifndef TRACKING_HPP
#define TRACKING_HPP

#include "LDPMaths.hpp"

using namespace LDPMaths;

// Structure to represent the LDP's camera head. Contains all the data 
// necessary to drive an IG and simulate the real head position for masking
// calculations


struct RangeData
{
   RangeData() : slantRange(0.0), planRange(0.0), heightAboveTarget(0.0) {}

   LDPMaths::NavCosines cosines;
   double               slantRange;
   double               planRange;
   double               heightAboveTarget;
};



struct OwnshipData
{
   LDPMaths::EarthPosition    position;
   LDPMaths::Attitude         attitude;

   LDPMaths::BodyVelocity     velocityBody;
   LDPMaths::BodyAcceleration accelerationBody;

   LDPMaths::NavVelocity      velocityNav;
   LDPMaths::NavAcceleration  accelerationNav;

   LDPMaths::RotationMatrix   navToBodyRotation;
   LDPMaths::RotationMatrix   bodyToNavRotation;
};



struct CameraData
{
   CameraData() : slantRange(0.0), FoV(0.0), bezelRoll(0.0) {}

   EarthPosition    eyePoint;  // Camera's position in the world, in
                               // latitude, longitude and altitude above
                               // WGS-84 sea-level. Raidans and metres.

   AzimuthElevation anglesNav;   // Camera's orientation relative to the 
                                 // horizon. Radians.
                                 
   AzimuthElevation anglesBody;  // Camera's orientation relative to the
                                 // ownship's body. Radians.

   double           slantRange;  // Slant range from the camera's position
                                 // to the target. Metres.
                                 
   double FoV;  // The camera's horizontal and vertical FoV, in radians

   double bezelRoll;    // The amount of roll which would be applied to 
                        // horizon stabilise the camera's image

   EarthPosition    starePoint;

   bool             slewInProgress;


};

static const CameraData defaultCamera;

CameraData lookAtLatLongAlt(EarthPosition    target,
                            EarthPosition    ownshipPosition,
                            Attitude         ownshipAttitude,
                            double           commandedFoV,
                            bool             limitCameraRange,
                            float            maxCameraRange,
                            AzimuthElevation slewDemand,
                            double           dt);



#endif
