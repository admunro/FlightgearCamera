// ----------------------------------------------------------------------------
//
// File:  Tracking.cpp
//
//
// Company    :   Cassidian 
//
// Department :   COESS3
//
// Copyright (C) 2008 by EADS
//
// Project    :   ITD
// Part       :   LDP
// Modul      :   ACS
//
// Created by :   Munro   Date: Mi 20 March 2013 
// //
// Description:   
// 
// This file contains algorithms for pointing the head of a camera mounted on
// an aircraft at a particular point on the ground, and for moving the camera's
// point of interest around.
//
// This is designed to provide the functionality required for Slave and Manual
// Tracking of the Litening III Laser Designator Pod at the P1E(b) EF standard.
//
// ----------------------------------------------------------------------------
//
#include <cmath>
#include <iostream>
#include "Tracking.hpp"

const float XYDeadzone       = 0.00;


// Returns the azimuth and elevation in radians of the camera head position
// mounted on ownship
CameraData lookAtLatLongAlt(EarthPosition    targetPosition,
                            EarthPosition    ownshipPosition,
                            Attitude         ownshipAttitude,
                            double           commandedFoV,
                            AzimuthElevation slewDemand,
                            double           dt)
{
   static const double maxCameraRange = 100000;
   

   CameraData camera;
   NavCosines targetNavCosines;

   NavPosition targetNavPosition = latLongToNED(ownshipPosition, 
                                                targetPosition);

   camera.anglesNav = getAngles(ownshipPosition, targetPosition);

   Cartesian ownshipXYZ = LLH2XYZ(ownshipPosition);
   Cartesian targetXYZ  = LLH2XYZ(targetPosition);
   Cartesian eyepointXYZ;

   GeocentricRange ownshipToTarget;

   ownshipToTarget.X = ownshipXYZ.X - targetXYZ.X;
   ownshipToTarget.Y = ownshipXYZ.Y - targetXYZ.Y;
   ownshipToTarget.Z = ownshipXYZ.Z - targetXYZ.Z;

   double aircraftSlantRange = getSlantRange(ownshipXYZ, targetXYZ);

   if ((fabs(slewDemand.azimuth) > XYDeadzone) ||
	   (fabs(slewDemand.elevation) > XYDeadzone))
   {
	   // Azimuth is has a range of +/- 180
	   if (camera.anglesNav.azimuth * RAD_2_DEG < 0)
	   {
	      camera.anglesNav.azimuth -= slewDemand.azimuth   * commandedFoV * dt;
	   }
	   else
	   {
          camera.anglesNav.azimuth += slewDemand.azimuth   * commandedFoV * dt;
	   }

	   camera.anglesNav.elevation += slewDemand.elevation * commandedFoV * dt;

	   targetNavCosines = navAnglesToCosines(camera.anglesNav);

	   targetNavPosition = newNavPosition(targetNavCosines, aircraftSlantRange);

	   camera.starePoint = newEarthPosition(ownshipPosition, targetNavPosition);

   }
   else
   {
	   camera.starePoint = targetPosition;
   }

   targetNavCosines = navAnglesToCosines(camera.anglesNav);

   RotationMatrix navToBodyRotation = getNavToBodyRotation(ownshipAttitude);

   BodyCosines targetBodyCosines = navCosinesToBodyCosines(targetNavCosines,
		                                                   navToBodyRotation);

   camera.anglesBody = bodyCosinesToAngles(targetBodyCosines);
                                        
   double k = 0;

   if (aircraftSlantRange > maxCameraRange)
   {
      k = (aircraftSlantRange - maxCameraRange) / aircraftSlantRange;

      camera.FoV = 2.0 * atan(aircraftSlantRange / 
                              maxCameraRange * tan(commandedFoV / 2.0));

      eyepointXYZ.X = ownshipXYZ.X - ownshipToTarget.X * k;  
      eyepointXYZ.Y = ownshipXYZ.Y - ownshipToTarget.Y * k;  
      eyepointXYZ.Z = ownshipXYZ.Z - ownshipToTarget.Z * k;

      camera.eyePoint = XYZ2LLH(eyepointXYZ);

   }
   else
   {
      eyepointXYZ       = ownshipXYZ;

      camera.FoV        = commandedFoV;
      camera.eyePoint   = ownshipPosition;
   }

   camera.anglesNav = getAngles(camera.eyePoint, targetPosition);

   return camera;
   
}



