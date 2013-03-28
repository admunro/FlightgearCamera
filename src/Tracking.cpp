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
#include <cstdio>
#include <iostream>
#include "Tracking.hpp"

const float XYDeadzone       = 0.00;


// Returns the azimuth and elevation in radians of the camera head position
// mounted on ownship
CameraData lookAtLatLongAlt(EarthPosition    targetPosition,
                            EarthPosition    ownshipPosition,
                            Attitude         ownshipAttitude,
                            double           commandedFoV,
                            bool             limitCameraRange,
                            float            maxCameraRange,
                            AzimuthElevation slewDemand,
                            double           dt)
{
   
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

   //double aircraftSlantRange = getSlantRange(ownshipXYZ, targetXYZ);
   camera.slantRange = getSlantRange(ownshipXYZ, targetXYZ);


   targetNavCosines = navAnglesToCosines(camera.anglesNav);

   RotationMatrix navToBodyRotation = getNavToBodyRotation(ownshipAttitude);

   BodyCosines targetBodyCosines = navCosinesToBodyCosines(targetNavCosines,
		                                                   navToBodyRotation);

   camera.anglesBody = bodyCosinesToAngles(targetBodyCosines);
                                        
   double k = 0;

   if ((camera.slantRange > maxCameraRange) && limitCameraRange)
   {
      k = (camera.slantRange - maxCameraRange) / camera.slantRange;

      camera.FoV = 2.0 * atan(camera.slantRange /
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

   camera.slewInProgress = fabs(slewDemand.azimuth)   > XYDeadzone ||
                           fabs(slewDemand.elevation) > XYDeadzone;

   if (camera.slewInProgress)
   {
      double azimuthangle_eyepoint_geoid = 
        camera.anglesNav.azimuth + commandedFoV * dt * slewDemand.azimuth;

      double elevationangle_eyepoint_geoid = 
        camera.anglesNav.elevation + commandedFoV * dt * slewDemand.elevation;

      // Add the demanded slew to the azimuth and elevation angles calculated
      // for the target
      camera.anglesNav.azimuth   += commandedFoV * dt * slewDemand.azimuth;
      camera.anglesNav.elevation += commandedFoV * dt * slewDemand.elevation;
      
      // new target position estimation based on constant altitude geoid 
      NavPosition targetUnity;

      targetUnity.N = -cos(azimuthangle_eyepoint_geoid) * 
                       cos(elevationangle_eyepoint_geoid);

      targetUnity.E = -sin(azimuthangle_eyepoint_geoid) * 
                       cos(elevationangle_eyepoint_geoid);

      targetUnity.D = -sin(elevationangle_eyepoint_geoid);
      
  
      camera.starePoint = targetPosition; 
       
      double k = (ownshipPosition.alt - camera.starePoint.alt) / 
                  sin(elevationangle_eyepoint_geoid);

      double resolution = 1000.0;
      double sw = 0.66;

      
      NavPosition   tmp_Navpos;
      Cartesian     tmp_XYZpos;
      EarthPosition tmp_TargetLLH;
      
      while (resolution > 0.001 && sw > 0.00001)  
      {    
         tmp_Navpos.N =  targetUnity.N * k;
         tmp_Navpos.E =  targetUnity.E * k;
         tmp_Navpos.D = -targetUnity.D * k;

         tmp_XYZpos     = ENU2XYZ(tmp_Navpos, eyepointXYZ);

         tmp_TargetLLH  = XYZ2LLH(tmp_XYZpos);

         if (tmp_TargetLLH.alt > camera.starePoint.alt) 
         {
            k = k * (1.0 + sw) ;
         }
         if (tmp_TargetLLH.alt < camera.starePoint.alt) 
         {
            k = k * (1.0 - sw) ;
         }
         sw = sw * 0.7;

         resolution = fabs(tmp_TargetLLH.alt - camera.starePoint.alt);
      }
  
      camera.starePoint = XYZ2LLH(tmp_XYZpos);  

   }
   else
   {
	   camera.starePoint = targetPosition;
   }

   return camera;
   
}


