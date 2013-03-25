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
#include "wgs84_utils.h"

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
   static const double maxCameraRange = 5000;
   

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


   if ((fabs(slewDemand.azimuth) > XYDeadzone) ||
	   (fabs(slewDemand.elevation) > XYDeadzone))
   {
      
      double pos_LOS_xyz[3];
      pos_LOS_xyz[0] = eyepointXYZ.X;
      pos_LOS_xyz[1] = eyepointXYZ.Y;
      pos_LOS_xyz[2] = eyepointXYZ.Z;



      double azimuthangle_eyepoint_geoid = 
        camera.anglesNav.azimuth + commandedFoV * dt * slewDemand.azimuth;

      double elevationangle_eyepoint_geoid = 
        camera.anglesNav.elevation + commandedFoV * dt * slewDemand.elevation;

      // Add the demanded slew to the azimuth and elevation angles calculated
      // for the target
      camera.anglesNav.azimuth   += commandedFoV * dt * slewDemand.azimuth;
      camera.anglesNav.elevation += commandedFoV * dt * slewDemand.elevation;

      //NavPosition targetUnity = newNavPosition(navAnglesToCosines(camera.anglesNav), 1.0);   THIS IS THE PROBLEM!
      
      // new target position estimation based on constant altitude geoid 
      NavPosition targetUnity;
      targetUnity.N = -sin(azimuthangle_eyepoint_geoid) * 
                       cos(elevationangle_eyepoint_geoid);

      targetUnity.E = -cos(azimuthangle_eyepoint_geoid) * 
                       cos(elevationangle_eyepoint_geoid);

      targetUnity.D = -sin(elevationangle_eyepoint_geoid);
      
  
      camera.starePoint = targetPosition; 
       
      double k = (ownshipPosition.alt - camera.starePoint.alt) / 
                  sin(elevationangle_eyepoint_geoid);

      double resolution = 1000.0;
      double sw = 0.66;

      double LOS_enu_azel_tmp[3];
      double pos_tgt_xyz_tmp[3];
      
      NavPosition   tmp_Navpos;
      Cartesian     tmp_XYZpos;
      Cartesian     starePointXYZ;
      EarthPosition tmp_TargetLLH;
      
      double pos_tgt_llh_tmp[3];
  
      while (resolution > 0.001 && sw > 0.00001)  
      {    
         LOS_enu_azel_tmp[0] = targetUnity.N * k;
         LOS_enu_azel_tmp[1] = targetUnity.E * k;
         LOS_enu_azel_tmp[2] = targetUnity.D * k;

         // Andy's version of ENU2XYZ doesn't work, missing parameters of the
         // origin. Isn't used in tracking calcs, which is why it wasn't 
         // causing trouble before. 

         wgs84ENU2XYZ(&LOS_enu_azel_tmp[0], &LOS_enu_azel_tmp[1], &LOS_enu_azel_tmp[2], &pos_LOS_xyz[0], &pos_LOS_xyz[1], &pos_LOS_xyz[2], &pos_tgt_xyz_tmp[0], &pos_tgt_xyz_tmp[1], &pos_tgt_xyz_tmp[2]);
         wgs84XYZ2LLH(&pos_tgt_xyz_tmp[0], &pos_tgt_xyz_tmp[1], &pos_tgt_xyz_tmp[2], &pos_tgt_llh_tmp[0], &pos_tgt_llh_tmp[1], &pos_tgt_llh_tmp[2]); 

         if (pos_tgt_llh_tmp[2] > camera.starePoint.alt) 
         {
            k = k * (1.0 + sw) ;
         }
         if (pos_tgt_llh_tmp[2] < camera.starePoint.alt) 
         {
            k = k * (1.0 - sw) ;
         }
         sw = sw * 0.7;
         resolution = fabs(pos_tgt_llh_tmp[2] - camera.starePoint.alt);
      }
  
   
      starePointXYZ.X = pos_tgt_xyz_tmp[0];
      starePointXYZ.Y = pos_tgt_xyz_tmp[1];
      starePointXYZ.Z = pos_tgt_xyz_tmp[2];
     
      camera.starePoint = XYZ2LLH(starePointXYZ);  

   }
   else
   {
	   camera.starePoint = targetPosition;
   }

   return camera;
   
}


