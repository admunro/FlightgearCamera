/*
 * LDPMaths.hpp
 *
 *  Contains various structures and helper methods used by the LDP for
 *  positioning the LDP's camera, tracking etc
 *
 *  Created on: Nov 23, 2012
 *      Author: munro
 */

#ifndef LDPMATHS_HPP_
#define LDPMATHS_HPP_


namespace LDPMaths
{
   static const double PI = 3.14159265359;
   static const double R  = 6378137.0;

   static const double DEG_2_RAD = PI / 180.0;
   static const double RAD_2_DEG = 1.0 / DEG_2_RAD;
   static const double M_2_FT    = 3.28084;
   static const double FT_2_M    = 1 / M_2_FT;

   /*
    * Ownship data
    *
    *
    */
   struct Attitude
   {
      Attitude() : bank(0.0), inclination(0.0), heading(0.0) {}

      double bank;        // radians   +/- PI
      double inclination; // radians   +/- PI/2
      double heading;     // radians   0.. 2PI
   };

   struct AzimuthElevation
   {
      AzimuthElevation() : azimuth(0.0), elevation(0.0) {}
      AzimuthElevation(double az, double el) { azimuth = az; elevation = el; }

      double azimuth;    // radians
      double elevation;  // radians
   };


   /*
    * Coordinate systems - Earth referenced
    *
    */
   struct EarthPosition
   {
      EarthPosition() : lat(0.0), lon(0.0), alt(0.0) {}

      EarthPosition(double latitude, double longitude, double altitude)
      {
    	  lat = latitude;
    	  lon = longitude;
    	  alt = altitude;
      }

      double lat; // radians
      double lon; // radians
      double alt; // metres
   };

   /*
    * Coordinate systems - Navigation axis referenced
    *
    * Axes North, East and Down from Aircraft centre of gravity
    *
    */
   struct NavPosition
   {
      NavPosition() : N(0.0), E(0.0), D(0.0) {}

      double N; // metres
      double E; // metres
      double D; // metres
   };

   struct NavVelocity
   {
      NavVelocity() : N(0.0), E(0.0), D(0.0) {}

      double N; // m/s
      double E; // m/s
      double D; // m/s
   };

   struct NavAcceleration
   {
      NavAcceleration() : N(0.0), E(0.0), D(0.0) {}

      double N; // m/s^^2
      double E; // m/s^^2
      double D; // m/s^^2
   };

   struct NavCosines
   {
      NavCosines() : N(0.0), E(0.0), D(0.0) {}

      double N;
      double E;
      double D;
   };


   /*
    * Coordinate systems - aircraft body referenced
    *
    * Axes X, Y and Z, longitudonal, normal and down from centre of gravity
    *
    */
   struct BodyPosition
   {
      BodyPosition() : X(0.0), Y(0.0), Z(0.0) {}

      double X; // metres
      double Y; // metres
      double Z; // metres
   };

   struct BodyVelocity
   {
      BodyVelocity() : X(0.0), Y(0.0), Z(0.0) {}

      double X; // metres/second
      double Y; // metres/second
      double Z; // metres/second
   };

   struct BodyAcceleration
   {
      BodyAcceleration() : X(0.0), Y(0.0), Z(0.0) {}

      double X; // metres/second^^2
      double Y; // metres/second^^2
      double Z; // metres/second^^2
   };

   struct BodyCosines
   {
      BodyCosines() : X(0.0), Y(0.0), Z(0.0) {}

      double X;
      double Y;
      double Z;
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

   Cartesian ENU2XYZ(NavPosition nav, Cartesian geoc);

   NavPosition XYZ2ENU(Cartesian geoc1, 
                       Cartesian geoc2);
   


}


#endif /* LDPMATHS_HPP_ */
