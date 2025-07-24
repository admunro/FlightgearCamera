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


   /*
    * Method declarations
    *
    */
   AzimuthElevation bodyCosinesToAngles(BodyCosines cosines);
   AzimuthElevation getAngles(EarthPosition origin, EarthPosition poi);
   AzimuthElevation getAngles(BodyPosition bodyPosn);
   AzimuthElevation getAngles(NavPosition  navPosn);
   AzimuthElevation navCosinesToAngles(NavCosines cosines);


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


#endif /* LDPMATHS_HPP_ */
