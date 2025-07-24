from math import sin
from math import cos
from math import atan
from math import atan2
from math import sqrt

#####################################################################
class Attitude:
    
    def __init__(self, bank = 0.0, inclination = 0.0, heading = 0.0):

        self.bank        = bank        # radians   +/- PI
        self.inclination = inclination # radians   +/- PI/2
        self.heading     = heading     # radians   0.. 2PI

#####################################################################
class AzimuthElevation:

    def __init__(self, azimuth = 0.0, elevation = 0.0):

        self.azimuth   = azimuth   # radians
        self.elevation = elevation # radians

#####################################################################
class Earth:

    def __init__(self, latitude = 0.0, longitude = 0.0, altitude = 0.0):

        self.lat = 0.0 # radians
        self.lon = 0.0 # radians
        self.alt = 0.0 # metres

#####################################################################
class Nav:

    def __init__(self, N = 0.0, E = 0.0, D = 0.0):

        self.N = 0.0 # metres
        self.E = 0.0 # metres
        self.D = 0.0 # metres

#####################################################################
class Cartesian:

    def __init__(self, X = 0.0, Y = 0.0, Z = 0.0):

        self.X = X
        self.Y = Y
        self.Z = Z


#####################################################################
class RotationMatrix:

   def __init__(self,
                C_XX = 0.0, 
                C_XY = 0.0, 
                C_XZ = 0.0,
                C_YX = 0.0, 
                C_YY = 0.0, 
                C_YZ = 0.0,
                C_ZX = 0.0, 
                C_ZY = 0.0, 
                C_ZZ = 0.0):

      self.C_XX = C_XX
      self.C_XY = C_XY
      self.C_XZ = C_XZ
      self.C_YX = C_YX
      self.C_YY = C_YY
      self.C_YZ = C_YZ
      self.C_ZX = C_ZX
      self.C_ZY = C_ZY
      self.C_ZZ = C_ZZ


#####################################################################
class OmegaMatrix:

   def __init__(self,
                omega_11 = 0.0, 
                omega_12 = 0.0, 
                omega_13 = 0.0, 
                omega_21 = 0.0, 
                omega_22 = 0.0, 
                omega_23 = 0.0, 
                omega_31 = 0.0, 
                omega_32 = 0.0, 
                omega_33 = 0.0):
   
      self.omega_11 = omega_11 
      self.omega_12 = omega_12 
      self.omega_13 = omega_13
      self.omega_21 = omega_21 
      self.omega_22 = omega_22 
      self.omega_23 = omega_23
      self.omega_31 = omega_31 
      self.omega_32 = omega_32 
      self.omega_33 = omega_33



    # ------------------------------------------------------------------------
    # getAngles(earthPosOrigin, earthPos)
    # 
    # Returns the azimuth and elevation of the poi from the source, in Nav
    # axis 
    # 
    # ------------------------------------------------------------------------
    def getAngles(earthPosOrigin, earthPosPOI):

        navAngles = AzimuthElevation()

        navPosition = NavAngles.get(earthPosOrigin, earthPosPOI)

        return getAnglesFromNav(navPosition)



    # --------------------------------------------------------------------------
    # getAngles(bodyPosition)
    #
    # Returns the supplied Body Axis position as an azimuth and elevation
    #
    # --------------------------------------------------------------------------
    def getAnglesFromBody(bodyPosition):
        angles = AzimuthElevation()

        angles.azimuth = atan2(bodyPosition.Y, bodyPosition.X)

        angles.elevation = \
          atan2(- bodyPosition.Z,
                     sqrt(bodyPosition.X ** 2 + bodyPosition.Y ** 2 ))

        return angles


    # --------------------------------------------------------------------------
    # AzimuthElevation getAngles(NavAxisPosition navPosn)
    #
    # Returns the supplied Navigation Axis position as an azimuth and elevation
    #
    # --------------------------------------------------------------------------
    def getAnglesFromNav(navPosition):
        angles = AzimuthElevation()

        angles.azimuth = atan2(navPosition.E, navPosition.N)

        angles.elevation = atan2( -navPosition.D, 
                                  sqrt(navPosition.N ** 2 + 
                                            navPosition.E ** 2))

        return angles






    # --------------------------------------------------------------------------
    # 
    # Computes a new set of lat/long coordinates given a starting position and
    # a displacement in North/East/Down.
    # 
    # --------------------------------------------------------------------------
    def get(earthPositionStart, navPosition):

        finalPosition = EarthPosition()

        bearing  = atan(navPosition.E / navPosition.N)
        distance = sqrt(navPosition.N ** 2 + navPosition.E ** 2)
        d_over_R = distance / R
        
        finalPosition.lat = \
          asin(sin(earthPositionStart.lat) * cos(d_over_R) + 
          cos(earthPositionStart.lat) * sin(d_over_R) * cos(bearing)

        finalPosition.lon = \
          atan2(sin(bearing) * sin(d_over_R) * \
                cos(earthPositionStart.lat),
                sin(d_over_R) * cos(bearing))

        finalPosition.alt = earthPositionStart.alt - navPosition.D

        return finalPosition


    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    def getSimple(earthPositionStart, navPosition):
        
        finalPosition = EarthPosition()

        finalPosition.lat = earthPositionStart.lat + navPosition.N / R

        finalPosition.lon = \
          earthPositionStart.lon +
          navPosition.E / (R * cos(navPositionStart.lat))

        finalPosition.alt = earthPositionStart.alt

        return finalPosition

    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    def getNewPosition(origin, speed, bearing, dt):

        EarthPosition dest

        d = speed * dt

        dest.lat = asin( sin(origin.lat) * cos(d/R)) + 
                   cos(origin.lat) * sin(d/R)* cos(bearing)

        dest.lon = origin.lon + \
                   atan2(sin(bearing)*sin(d/R)*cos(origin.lat),
                              cos(d/R) - sin(origin.lat) * sin(dest.lat))

        dest.alt = origin.alt

        return dest




class BodyPosition:

    def __init__(self):
       self.X = 0.0 # metres
       self.Y = 0.0 # metres
       self.Z = 0.0 # metres

    def get(navPosition, rotationMatrix):
       pass
        

class BodyVelocity:

    def __init__(self):
       self.X = 0.0 # metres / second
       self.Y = 0.0 # metres / second
       self.Z = 0.0 # metres / second

class BodyAcceleration:

    def __init__(self):
       self.X = 0.0 # metres / second ^^ 2
       self.Y = 0.0 # metres / second ^^ 2
       self.Z = 0.0 # metres / second ^^ 2

class BodyCosines:

    def __init__(self):
       self.X = 0.0 
       self.Y = 0.0 
       self.Z = 0.0 

    def get(navCosines, rotationMatrix):
        pass

    def get(angle):
        pass
       

#
# Coordinate transformation data types
#






def getImageRoll(attitude, camera):
    pass

def getSlantRange(navPosition):
    pass

