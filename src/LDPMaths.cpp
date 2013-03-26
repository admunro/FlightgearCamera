/*
 * LDPMaths.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: munro
 */
#include "LDPMaths.hpp"
#include <cmath>
#include <iostream>

namespace LDPMaths
{


   // --------------------------------------------------------------------------
   // AzimuthElevation getAngles(BodyAxisPosition bodyPosn)
   //
   // Returns the supplied Body Axis position as an azimuth and elevation
   //
   // --------------------------------------------------------------------------
   AzimuthElevation getAngles(BodyPosition bodyPosn)
   {
      AzimuthElevation angles;

      angles.azimuth = atan2( bodyPosn.Y, bodyPosn.X );

      angles.elevation = atan2( - bodyPosn.Z,
                                sqrt(pow(bodyPosn.X, 2) + pow(bodyPosn.Y, 2)) );

      return angles;

   }


   // --------------------------------------------------------------------------
   // AzimuthElevation getAngles(NavAxisPosition navPosn)
   //
   // Returns the supplied Navigation Axis position as an azimuth and elevation
   //
   // --------------------------------------------------------------------------
   AzimuthElevation getAngles(NavPosition posn)
   {
      AzimuthElevation angles;

      angles.azimuth = atan2(posn.E, posn.N);

      angles.elevation = atan2( -posn.D,
                                sqrt(pow(posn.N, 2) + pow(posn.E, 2)) );

      return angles;

   }


   // --------------------------------------------------------------------------
   // AzimuthElevation getAngles(EarthPosition source, EarthPosition poi)
   //
   // Returns the azimuth and elevation of the poi from the source, in Nax
   // axis 
   //
   // --------------------------------------------------------------------------
   AzimuthElevation getAngles(EarthPosition source, EarthPosition poi)
   {
       NavPosition      navPosition;
       AzimuthElevation newAngles;

       navPosition = latLongToNED(source, poi);
       newAngles =   getAngles(navPosition);

      return newAngles;
   }





   NavPosition latLongToNED(EarthPosition position1, EarthPosition position2)
   {
      NavPosition position;

      double A_ELLIPSOID = R;
      double E_SQUARE_2  = 0.00669438;
      double A_ELL_E_SQR = A_ELLIPSOID * E_SQUARE_2;

      double TRAFO_VEC_1 = 0.0;
      //double TRAFO_VEC_2 = 0.0;
      double TRAFO_VEC_3 = 0.0;

      double sin_lat1 = sin(position1.lat);
      double cos_lat1 = cos(position1.lat);
      double sin_lat2 = sin(position2.lat);
      double cos_lat2 = cos(position2.lat);

      OmegaMatrix omega;

      omega = getOmegaMatrix(position1.lat,
                             position1.lon,
                             position2.lat,
                             position2.lon);

      TRAFO_VEC_1 =
            - A_ELL_E_SQR * sin_lat2 * cos_lat2
            / sqrt( 1.0 - E_SQUARE_2 * pow(sin_lat2,2) ) ;

      TRAFO_VEC_3 =
        - A_ELLIPSOID * sqrt ( 1.0 - E_SQUARE_2 * pow(sin_lat2, 2) )
        - position2.alt;


      position.N = omega.omega_11 * TRAFO_VEC_1 +
                   omega.omega_13 * TRAFO_VEC_3 +
                   ( A_ELL_E_SQR * sin_lat1 * cos_lat1 /
                     sqrt(1.0 - E_SQUARE_2 * pow(sin_lat1,2)) );

      position.E = omega.omega_21 * TRAFO_VEC_1 +
                   omega.omega_23 * TRAFO_VEC_3;

      position.D = omega.omega_31 * TRAFO_VEC_1 +
                   omega.omega_33 * TRAFO_VEC_3 +
                   A_ELLIPSOID * sqrt(1.0 - E_SQUARE_2 * pow(sin_lat1, 2)) +
                   position1.alt;

      return position;
   }

   NavPosition latLongToNEDSimple(EarthPosition position1,
                                  EarthPosition position2)
   {
      NavPosition position;

      position.N = (position2.lat - position1.lat) * R;
      position.E = (position2.lon - position1.lon) * R * cos(position1.lat);
      position.D = position1.alt - position2.alt;

      return position;
   }


   NavPosition newNavPosition(NavCosines cosines,
                            double     slantRange)
   {
      NavPosition newPosition;

      newPosition.N = slantRange * cosines.N;
      newPosition.E = slantRange * cosines.E;
      newPosition.D = slantRange * cosines.D;

      return newPosition;

   }



   NavPosition newNavPosition(NavPosition     initialPosition,
                              NavVelocity     velocity,
                              NavAcceleration acceleration,
                              float           dt)
   {
      NavPosition newPosition;

      newPosition.N = initialPosition.N +
                      velocity.N * dt +
                      (acceleration.N * pow(dt, 2) / 2.0 );

      newPosition.E = initialPosition.E +
                      velocity.E * dt +
                      (acceleration.E * pow(dt, 2) / 2.0 );

      newPosition.D = initialPosition.D +
                      velocity.D * dt +
                      (acceleration.D * pow(dt, 2) / 2.0 );

      return newPosition;

   }



   NavCosines navAnglesToCosines(AzimuthElevation navAngles)
   {
      NavCosines cosines;

      double cosAzimuth   = cos(navAngles.azimuth);
      double sinAzimuth   = sin(navAngles.azimuth);

      double cosElevation = cos(navAngles.elevation);
      double sinElevation = sin(navAngles.elevation);

      cosines.N = cosElevation * cosAzimuth;
      cosines.E = cosElevation * sinAzimuth;
      cosines.D = -sinElevation;

      return cosines;
   }

   AzimuthElevation navCosinesToAngles(NavCosines cosines)
   {
      AzimuthElevation angles;

      angles.elevation = -asin(cosines.D);
      angles.azimuth = atan2(cosines.E, cosines.N);

      return angles;
   }


   NavVelocity toNavVelocity(BodyVelocity   bodyVelocity,
                             RotationMatrix rotation)
   {
      NavVelocity velocity;

      velocity.N = rotation.C_XX * bodyVelocity.X +
                   rotation.C_XY * bodyVelocity.Y +
                   rotation.C_XZ * bodyVelocity.Z;

      velocity.E = rotation.C_YX * bodyVelocity.X +
                   rotation.C_YY * bodyVelocity.Y +
                   rotation.C_YZ * bodyVelocity.Z;

      velocity.D = rotation.C_ZX * bodyVelocity.X +
                   rotation.C_ZY * bodyVelocity.Y +
                   rotation.C_ZZ * bodyVelocity.Z;

      return velocity;
   }


   /*
    * Converts the supplied Body Axis acceleration (X, Y and Z) into Navigation
    * Axis (North, East and Down)
    *
    */
   NavAcceleration toNavAcceleration(BodyAcceleration bodyAxis,
                                     RotationMatrix       rotation)
   {
      NavAcceleration acceleration;

      acceleration.N = rotation.C_XX * bodyAxis.X +
                       rotation.C_XY * bodyAxis.Y +
                       rotation.C_XZ * bodyAxis.Z;

      acceleration.E = rotation.C_YX * bodyAxis.X +
                       rotation.C_YY * bodyAxis.Y +
                       rotation.C_YZ * bodyAxis.Z;

      acceleration.D = rotation.C_ZX * bodyAxis.X +
                       rotation.C_ZY * bodyAxis.Y +
                       rotation.C_ZZ * bodyAxis.Z;

      return acceleration;

   }


   // --------------------------------------------------------------------------
   // BodyAxisPosition navToBodyPosition()
   //
   // Translates a position in Navigation axes (range N, E, D) into the
   // equivalent in aircraft body axes (range longitudonal, transverse and
   // normal).
   //
   // --------------------------------------------------------------------------
   BodyPosition navToBodyPosition(NavPosition navPos,
                                  RotationMatrix rotation)
   {
      BodyPosition bodyPosn;

      bodyPosn.X = rotation.C_XX * navPos.N +
                   rotation.C_YX * navPos.E +
                   rotation.C_ZX * navPos.D;

      bodyPosn.Y = rotation.C_XY * navPos.N +
                   rotation.C_YY * navPos.E +
                   rotation.C_ZY * navPos.D;

      bodyPosn.Z = rotation.C_XZ * navPos.N +
                   rotation.C_YZ * navPos.E +
                   rotation.C_ZZ * navPos.D;

      return bodyPosn;

   }

   RotationMatrix transposeMatrix(RotationMatrix original)
   {
      RotationMatrix transpose;

      transpose.C_XX = original.C_XX;
      transpose.C_XY = original.C_YX;
      transpose.C_XZ = original.C_ZX;

      transpose.C_YX = original.C_XY;
      transpose.C_YY = original.C_YY;
      transpose.C_YZ = original.C_ZY;

      transpose.C_ZX = original.C_XZ;
      transpose.C_ZY = original.C_YZ;
      transpose.C_ZZ = original.C_ZZ;

      return transpose;
   }



   BodyCosines navCosinesToBodyCosines(NavCosines     navCosines,
                                       RotationMatrix rotation)
   {
      BodyCosines bodyCosines;

      double result_x, result_y, result_z, norm;


      result_x = rotation.C_XX * navCosines.N +
                 rotation.C_YX * navCosines.E +
                 rotation.C_ZX * navCosines.D;

      result_y = rotation.C_XY * navCosines.N +
                 rotation.C_YY * navCosines.E +
                 rotation.C_ZY * navCosines.D;

      result_z = rotation.C_XZ * navCosines.N +
                 rotation.C_YZ * navCosines.E +
                 rotation.C_ZZ * navCosines.D;

      norm = sqrt(pow(result_x, 2) + pow(result_y, 2) + pow(result_z, 2));

      if (norm == 0.0)
      {
         bodyCosines.X = 1.0;
         bodyCosines.Y = 0.0;
         bodyCosines.Z = 0.0;
      }
      else
      {
         bodyCosines.X = result_x / norm;
         bodyCosines.Y = result_y / norm;
         bodyCosines.Z = result_z / norm;
      }

      return bodyCosines;
   }


   BodyCosines bodyAnglesToCosines(AzimuthElevation angles)
   {
      BodyCosines cosines;

      double cosAzimuth   = cos(angles.azimuth);
      double sinAzimuth   = sin(angles.azimuth);

      double cosElevation = cos(angles.elevation);
      double sinElevation = sin(angles.elevation);

      cosines.X =  cosElevation * cosAzimuth;
      cosines.Y =  cosElevation * sinAzimuth;
      cosines.Z = -sinElevation;

      return cosines;
   }


   NavCosines bodyCosinesToNavCosines(BodyCosines    bodyCosines,
                                      RotationMatrix rotation)
   {
      NavCosines navCosines;

      double result_x, result_y, result_z, norm;


      result_x = rotation.C_XX * bodyCosines.X +
                 rotation.C_XY * bodyCosines.Y +
                 rotation.C_XZ * bodyCosines.Z;

      result_y = rotation.C_YX * bodyCosines.X +
                 rotation.C_YY * bodyCosines.Y +
                 rotation.C_YZ * bodyCosines.Z;

      result_z = rotation.C_ZX * bodyCosines.X +
                 rotation.C_ZY * bodyCosines.Y +
                 rotation.C_ZZ * bodyCosines.Z;

      norm = sqrt(pow(result_x, 2) + pow(result_y, 2) + pow(result_z, 2));

      if (norm == 0.0)
      {
         navCosines.N = 1.0;
         navCosines.E = 0.0;
         navCosines.D = 0.0;
      }
      else
      {
         navCosines.N = result_x / norm;
         navCosines.E = result_y / norm;
         navCosines.D = result_z / norm;
      }

      return navCosines;
   }




   AzimuthElevation bodyCosinesToAngles(BodyCosines cosines)
   {
      AzimuthElevation bodyAngles;

      bodyAngles.elevation = -sin(cosines.Z);
      bodyAngles.azimuth   = atan2(cosines.Y, cosines.X);

      return bodyAngles;

   }

   // --------------------------------------------------------------------------
   //
   // Computes a new set of lat/long coordinates given a starting position and
   // a displacement in North/East/Down.
   //
   // --------------------------------------------------------------------------
   EarthPosition newEarthPosition(EarthPosition initialPosition,
                                  NavPosition   displacement)
   {
      EarthPosition finalPosition;

      double bearing = atan( displacement.E / displacement.N );

      double distance = sqrt(pow(displacement.N, 2) +
                             pow(displacement.E, 2));

      double d_over_R = distance/R;

      finalPosition.lat = asin(sin(initialPosition.lat) * cos(d_over_R) +
                               cos(initialPosition.lat) *
                               sin(d_over_R) *
                               cos(bearing));

      finalPosition.lon = initialPosition.lon +
                          atan2(sin(bearing) *
                                sin(d_over_R) *
                                cos(initialPosition.lat),

                                cos(d_over_R) -
                                sin(initialPosition.lat) * sin(finalPosition.lat) );

      finalPosition.alt = initialPosition.alt - displacement.D;

      return finalPosition;

   }


   EarthPosition newEarthPositionSimple(EarthPosition initialPosition,
                                        NavPosition   displacement)
   {
      EarthPosition finalPosition;

      finalPosition.lat = initialPosition.lat + displacement.N / R;

      finalPosition.lon = initialPosition.lon +
                          displacement.E / ( R * cos(initialPosition.lat) );

      finalPosition.alt = initialPosition.alt;

      return finalPosition;

   }



   // --------------------------------------------------------------------------
   // RotationMatrix getRotation(Attitude att)
   //
   // Returns a matrix which performs a conversion from Navigation to Body axes
   // --------------------------------------------------------------------------
   RotationMatrix getNavToBodyRotation(Attitude att)
   {

      double CPS = cos(att.heading);
      double SPS = sin(att.heading);

      double CPH = cos(att.bank);
      double SPH = sin(att.bank);

      double CTH = cos(att.inclination);
      double STH = sin(att.inclination);

      RotationMatrix rotation;

      rotation.C_XX = CPS * CTH;
      rotation.C_XY = CPS * STH * SPH - SPS * CPH;
      rotation.C_XZ = CPS * STH * CPH + SPS * SPH;

      rotation.C_YX = SPS * CTH;
      rotation.C_YY = SPS * STH * SPH + CPS * CPH;
      rotation.C_YZ = SPS * STH * CPH - CPS * SPH;

      rotation.C_ZX = -STH;
      rotation.C_ZY = CTH * SPH;
      rotation.C_ZZ = CTH * CPH;

      return rotation;

   }


   OmegaMatrix getOmegaMatrix(double Latitude_1,
                              double Longitude_1,
                              double Latitude_2,
                              double Longitude_2)
   {
      OmegaMatrix matrix;

      double sinLat1 = sin(Latitude_1);
      double cosLat1 = cos(Latitude_1);

      double sinLon1 = sin(Longitude_1);
      double cosLon1 = cos(Longitude_1);

      double sinLat2 = sin(Latitude_2);
      double cosLat2 = cos(Latitude_2);

      double sinLon2 = sin(Longitude_2);
      double cosLon2 = cos(Longitude_2);


      double cosLat1_cosLat2 = cosLat1 * cosLat2;
      double cosLat1_sinLat2 = cosLat1 * sinLat2;
      double sinLat1_cosLat2 = sinLat1 * cosLat2;
      double sinLat1_sinLat2 = sinLat1 * sinLat2;

      double cosLon1_cosLon2 = cosLon1 * cosLon2;
      double cosLon1_sinLon2 = cosLon1 * sinLon2;
      double sinLon1_cosLon2 = sinLon1 * cosLon2;
      double sinLon1_sinLon2 = sinLon1 * sinLon2;


      matrix.omega_11 = sinLat1_sinLat2 * cosLon1_cosLon2 +
                        sinLat1_sinLat2 * sinLon1_sinLon2 +
                        cosLat1_cosLat2;

      matrix.omega_12 = sinLat1 * (cosLon1_sinLon2 - sinLon1_cosLon2);

      matrix.omega_13 = sinLat1_cosLat2 * cosLon1_cosLon2 +
                        sinLat1_cosLat2 * sinLon1_sinLon2 -
                        cosLat1_sinLat2;

      matrix.omega_21 = sinLat2 * (sinLon1_cosLon2 - cosLon1_sinLon2);

      matrix.omega_22 = cosLon1_cosLon2 + sinLon1_sinLon2;

      matrix.omega_23 = cosLat2 * (sinLon1_cosLon2 - cosLon1_sinLon2);

      matrix.omega_31 = cosLat1_sinLat2 * cosLon1_cosLon2 +
                        cosLat1_sinLat2 * sinLon1_sinLon2 -
                        sinLat1_cosLat2;

      matrix.omega_32 = cosLat1 * (cosLon1_sinLon2 - sinLon1_cosLon2);

      matrix.omega_33 = cosLat1_cosLat2 * cosLon1_cosLon2 +
                        cosLat1_cosLat2 * sinLon1_sinLon2 +
                        sinLat1_sinLat2;

      return matrix;
   }

   // --------------------------------------------------------------------------
   //
   // Compute what the roll of the camera's body axis would be when converting
   // aircraft body angles to nav angles. This give the value by which the
   // LDP's camera must be rotated to horizon-stabilise its picture;
   //
   // --------------------------------------------------------------------------
   double getImageRoll(Attitude att, AzimuthElevation cam)
   {

      return asin(sin(cam.azimuth) * sin(att.inclination) +
                  cos(cam.azimuth) * cos(att.inclination) * sin(att.bank));

   }


   /*
    * Returns Slant range from the given position in North, East Down coords.
    *
    */
   double getSlantRange(NavPosition navPosition)
   {
      return sqrt(pow(navPosition.N, 2) +
                  pow(navPosition.E, 2) +
                  pow(navPosition.D, 2));
   }


   double getSlantRange(Cartesian from, Cartesian to)
   {
      return sqrt(pow(to.X - from.X, 2) + 
                  pow(to.Y - from.Y, 2) + 
                  pow(to.Z - from.Z, 2));
   }



      // WGS-84 Calculations - based on wgs84_utils.h, G. Kuenerth
      // WGS-84 LLH ENU XYZ Conversions
      static const double aa = 6378137.0000000000000000000;  // earth semimajor axis in meters
      static const double bb = 6356752.3142451793000000000;  // earth semiminor axis in meters [bb = a*(1.0 - 1.0/298.257223563)]
      static const double ee = 0.0818191908426215700;        // numeric excenticity            [ee = sqrt(1 - (b/a)*(b/a))]
      static const double a2 = 40680631590769.00000000000;   // [a2 = aa*aa]
      static const double b2 = 40408299984661.44500000000;   // [b2 = bb*bb]
      static const double e2 = 0.006694379990141329000000;   // [e2 = ee*ee]
      static const double pi = 3.1415926535897932384626433832795;


      Cartesian LLH2XYZ(EarthPosition geog)
      {
         Cartesian geoc;
         
         double sinLat = sin(geog.lat);
         double cosLat = cos(geog.lat);
         double sinLon = sin(geog.lon);
         double cosLon = cos(geog.lon);
 
         double tan2lat = pow(tan(geog.lat), 2);

         double tmp  = 1 - e2;
         double tmpden = sqrt(1 + tmp * tan2lat);
         double tmp2 = sqrt(1 - e2 * pow(sinLat, 2));

         geoc.X = (aa * cosLon) / tmpden + geog.alt * cosLon * cosLat;
         geoc.Y = (aa * sinLon) / tmpden + geog.alt * sinLon * cosLat;
         geoc.Z = (aa * tmp * sinLat) / tmp2 + geog.alt * sinLat;

         return geoc;

      }

      EarthPosition XYZ2LLH(Cartesian geoc) 
      {
        EarthPosition geog;

        double Z2 = pow(geoc.Z, 2);

        double ep = ee * (aa / bb);
        double r  = sqrt(pow(geoc.X, 2) + pow(geoc.Y, 2));
        double r2 = pow(r, 2);

        double E2 = a2 - b2;

        double F = 54 * b2 * Z2;
        double G = r2 + (1.0 - e2) * Z2 - e2 * E2;
        double c = (pow(e2, 2) * F * r2) / (pow(G, 3));

        double s = pow((1.0 + c + sqrt(pow(c, 2) + 2.0 * c)), 1.0 / 3.0);

        double P = F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * pow(G,2));
        double Q = sqrt(1.0 + 2.0 * pow(e2, 2) * P);

        double ro = -(P * e2 * r) / (1.0 + Q) + sqrt((a2 / 2.0) * (1.0 + 1.0 / Q) - (P
                     * (1.0 - e2) * Z2) / (Q * (1.0 + Q)) - P * r2 / 2.0);
   
        double  tmp = pow((r - e2 * ro), 2);

        double U = sqrt(tmp + Z2);
        double V = sqrt(tmp + (1.0 - e2) * Z2);
        double zo = (b2 * (geoc.Z)) / (aa * V);
        double temp = atan((geoc.Y) / (geoc.X));


        geog.alt = U * (1.0 - b2 / (aa * V));
        geog.lat = atan(((geoc.Z) + ep * ep * zo) / r);

        if ((geoc.X) >= 0)
        {
           (geog.lon) = temp;
        }
        else
        {
           if (((geoc.X) < 0) && ((geoc.Y) >= 0))
           {
              (geog.lon) = pi + temp;
           }
           else
           {
              (geog.lon) = temp - pi;
           }
       }

         return geog;

      }

      Cartesian ENU2XYZ(NavPosition nav, Cartesian origin) 
      {

         Cartesian neworigin;

         EarthPosition geog = XYZ2LLH(origin);

         double sinphi = sin(geog.lat);
         double cosphi = cos(geog.lat);
         double sinlam = sin(geog.lon);
         double coslam = cos(geog.lon);

         double R11 = -sinlam;
         double R12 = coslam;
         double R13 = 0.0;
         double R21 = -sinphi * coslam;
         double R22 = -sinphi * sinlam;
         double R23 = cosphi;
         double R31 = cosphi * coslam;
         double R32 = cosphi * sinlam;
         double R33 = sinphi;

         neworigin.X = origin.X + R11 * (nav.E) + R21 * (nav.N) + R31 * (-nav.D);
         neworigin.Y = origin.Y + R12 * (nav.E) + R22 * (nav.N) + R32 * (-nav.D);
         neworigin.Z = origin.Z + R13 * (nav.E) + R23 * (nav.N) + R33 * (-nav.D);

         return neworigin;

      }
      
      NavPosition XYZ2ENU(Cartesian geoc1, Cartesian geoc2)
      {
         NavPosition nav;

         EarthPosition geog2 = XYZ2LLH(geoc2);

         double sinphi = sin(geog2.lat);
         double cosphi = cos(geog2.lat);
         double sinlam = sin(geog2.lon);
         double coslam = cos(geog2.lon);

         double R11 = -sinlam;
         double R12 = coslam;
         double R13 = 0.0;
         double R21 = -sinphi * coslam;
         double R22 = -sinphi * sinlam;
         double R23 = cosphi;
         double R31 = cosphi * coslam;
         double R32 = cosphi * sinlam;
         double R33 = sinphi;
     
         double deltaX = geoc1.X - geoc2.X;
         double deltaY = geoc1.Y - geoc2.Y;
         double deltaZ = geoc1.Z - geoc2.Z;

         nav.E = R11 * (deltaX) + R12 * (deltaY) + R13 * (deltaZ);
         nav.N = R21 * (deltaX) + R22 * (deltaY) + R23 * (deltaZ);
         nav.D = (R31 * (deltaX) + R32 * (deltaY) + R33 * (deltaZ)) * -1.0;

         return nav;
      }

   EarthPosition newEarthPosition(EarthPosition origin,
                                  double speed, // metres
                                  double brng, // radians
                                  double dt)
   {
      EarthPosition dest;

      double d = speed * dt;

      dest.lat = asin( sin(origin.lat)*cos(d/R) + 
              cos(origin.lat)*sin(d/R)*cos(brng) );

      dest.lon = origin.lon + 
                       atan2(sin(brng)*sin(d/R)*cos(origin.lat), 
                             cos(d/R)-sin(origin.lat)*sin(dest.lat));

      dest.alt = origin.alt;

      return dest;

   }

}
