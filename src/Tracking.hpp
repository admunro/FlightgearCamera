// --------------------------------------------------------------------------------
//
// File:  Tracking.hpp
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
// 
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

struct PointOfInterest
{
   PointOfInterest() : slantRange(0.0),
                       heightAboveTarget(0.0) {}

   LDPMaths::EarthPosition    earthPosition;
   double                     slantRange;
   double                     heightAboveTarget;

   LDPMaths::NavPosition      positionNav;
   LDPMaths::NavCosines       cosinesNav;
   LDPMaths::AzimuthElevation anglesNav;

   LDPMaths::BodyPosition     positionBody;
   LDPMaths::BodyCosines      cosinesBody;
   LDPMaths::AzimuthElevation anglesBody;

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
   EarthPosition    eyePoint;  // Camera's position in the world, in
                               // latitude, longitude and altitude above
                               // WGS-84 sea-level. Raidans and metres.

   AzimuthElevation anglesNav;   // Camera's orientation relative to the 
                                 // horizon. Radians.
                                 
   AzimuthElevation anglesBody;  // Camera's orientation relative to the
                                 // ownship's body. Radians.
                                 

   double FoV;  // The camera's horizontal and vertical FoV, in radians

   double bezelRoll;    // The amount of roll which would be applied to 
                        // horizon stabilise the camera's image

   EarthPosition    starePoint;


};

CameraData lookAtLatLongAlt(EarthPosition    target,
                            EarthPosition    ownshipPosition,
                            Attitude         ownshipAttitude,
                            double           commandedFoV,
                            AzimuthElevation slewDemand,
                            double           dt);



#endif
