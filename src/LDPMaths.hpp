/*
 * LDPMaths.hpp
 *
 *  Contains various structures and helper methods used by the LDP for
 *  positioning the LDP's camera, tracking etc
 *
 *  Created on: Nov 23, 2012
 *      Author: munro
 */

#pragma once

namespace LDPMaths
{
   constexpr double PI = 3.14159265359;
   constexpr double R  = 6378137.0;

   constexpr double DEG_2_RAD = PI / 180.0;
   constexpr double RAD_2_DEG = 1.0 / DEG_2_RAD;
   constexpr double M_2_FT    = 3.28084;
   constexpr double FT_2_M    = 1 / M_2_FT;
   constexpr double KTS_2_M_S = 0.514444444;

   /*
    * Ownship data
    *
    *
    */
   struct Attitude
   {
      double bank {0.0};        // radians   +/- PI
      double inclination {0.0}; // radians   +/- PI/2
      double heading {0.0};     // radians   0.. 2PI
   };

   struct AzimuthElevation
   {
      AzimuthElevation() = default;
      AzimuthElevation(double az, double el) { azimuth = az; elevation = el; }

      double azimuth {0.0};    // radians
      double elevation {0.0};  // radians
   };

   /*
    * Coordinate systems - Earth referenced
    *
    */
   struct EarthPosition
   {
      EarthPosition() = default;
      
      EarthPosition(double latitude, double longitude, double altitude):
         lat(latitude), lon(longitude), alt(altitude) {}

      double lat {0.0}; // radians
      double lon {0.0}; // radians
      double alt {0.0}; // metres
   };

   /*
    * Coordinate systems - Navigation axis referenced
    *
    * Axes North, East and Down from Aircraft centre of gravity
    *
    */
   struct NavPosition
   {
      double N {0.0}; // metres
      double E {0.0}; // metres
      double D {0.0}; // metres
   };

   struct NavVelocity
   {
      double N {0.0}; // m/s
      double E {0.0}; // m/s
      double D {0.0}; // m/s
   };

   struct NavAcceleration
   {
      double N {0.0}; // m/s^^2
      double E {0.0}; // m/s^^2
      double D {0.0}; // m/s^^2
   };

   struct NavCosines
   {
      double N {0.0};
      double E {0.0};
      double D {0.0};
   };


   /*
    * Coordinate systems - aircraft body referenced
    *
    * Axes X, Y and Z, longitudonal, normal and down from centre of gravity
    *
    */
   struct BodyPosition
   {
      double X {0.0}; // metres
      double Y {0.0}; // metres
      double Z {0.0}; // metres
   };

   struct BodyVelocity
   {
      double X {0.0}; // metres/second
      double Y {0.0}; // metres/second
      double Z {0.0}; // metres/second
   };

   struct BodyAcceleration
   {
      double X {0.0}; // metres/second^^2
      double Y {0.0}; // metres/second^^2
      double Z {0.0}; // metres/second^^2
   };

   struct BodyCosines
   {
      double X {0.0};
      double Y {0.0};
      double Z {0.0};
   };


   /*
    * Coordinate transformation data types
    *
    */
   struct RotationMatrix
   {
      RotationMatrix() : C_XX(0.0), C_XY(0.0), C_XZ(0.0),
                         C_YX(0.0), C_YY(0.0), C_YZ(0.0),
                         C_ZX(0.0), C_ZY(0.0), C_ZZ(0.0)   {}

      double C_XX, C_XY, C_XZ;
      double C_YX, C_YY, C_YZ;
      double C_ZX, C_ZY, C_ZZ;
   };

   struct OmegaMatrix
   {
      OmegaMatrix() : omega_11(0.0), omega_12(0.0), omega_13(0.0),
                      omega_21(0.0), omega_22(0.0), omega_23(0.0),
                      omega_31(0.0), omega_32(0.0), omega_33(0.0)  {}

      double omega_11, omega_12, omega_13;
      double omega_21, omega_22, omega_23;
      double omega_31, omega_32, omega_33;
   };




   /*
    * Method declarations
    *
    */
   AzimuthElevation bodyCosinesToAngles(BodyCosines cosines);
   AzimuthElevation getAngles(EarthPosition origin, EarthPosition poi);
   AzimuthElevation getAngles(BodyPosition bodyPosn);
   AzimuthElevation getAngles(NavPosition  navPosn);


   RotationMatrix getNavToBodyRotation(Attitude att);
   RotationMatrix transposeMatrix(RotationMatrix original);


   OmegaMatrix getOmegaMatrix(double Latitude_1,
                              double Longitude_1,
                              double Latitude_2,
                              double Longitude_2);


   BodyPosition navToBodyPosition(NavPosition    navPos,
                                  RotationMatrix rotation);

   BodyCosines navCosinesToBodyCosines(NavCosines     angles,
                                       RotationMatrix rotation);

   BodyCosines bodyAnglesToCosines(AzimuthElevation angles);


   EarthPosition newEarthPosition(EarthPosition initialPosition,
                                  NavPosition   displacement);

   EarthPosition newEarthPositionSimple(EarthPosition initialPosition,
                                        NavPosition   displacement);


   NavCosines bodyCosinesToNavCosines(BodyCosines    bodyCosines,
                                      RotationMatrix rotation);

   NavCosines navAnglesToCosines(AzimuthElevation angles);

   AzimuthElevation navCosinesToAngles(NavCosines cosines);



   NavPosition latLongToNED(EarthPosition position1,
                            EarthPosition position2);

   NavPosition latLongToNEDSimple(EarthPosition position1,
                                  EarthPosition position2);

   NavPosition newNavPosition(NavCosines cosines,
		                      double     slantRange);

   NavPosition newNavPosition(NavPosition     initialPosition,
                              NavVelocity     initialVelocity,
                              NavAcceleration initialAcceleration,
                              float           dt);

   NavVelocity toNavVelocity(BodyVelocity   bodyAxis,
                             RotationMatrix rotation);

   NavAcceleration toNavAcceleration(BodyAcceleration bodyAxis,
                                     RotationMatrix   rotation);

   double getImageRoll(Attitude att, AzimuthElevation cam);

   double getSlantRange(NavPosition navPosition);


   struct Cartesian
   {
      Cartesian() : X(0.0), Y(0.0), Z(0.0) {}

      double X; // metres 
	   double Y; // metres
      double Z; // metres
   };

   struct GeocentricRange
   {
      GeocentricRange() : X(0.0), Y(0.0), Z(0.0) {}
      
      double X; // metres 
	   double Y; // metres
      double Z; // metres

   };


   double getSlantRange(Cartesian from, Cartesian to);

   Cartesian LLH2XYZ(EarthPosition earthPosition);
   EarthPosition XYZ2LLH(Cartesian geoc);

   Cartesian ENU2XYZ(NavPosition nav, Cartesian newPosition);

   NavPosition XYZ2ENU(Cartesian geoc1, 
                       Cartesian geoc2);


   EarthPosition newEarthPosition(EarthPosition origin,
                                  double speed, // metres
                                  double bearing, // radians
                                  double dt); 

   
}