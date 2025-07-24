from math import sin
from math import cos
from math import atan
from math import atan2
from math import sqrt

class AzimuthElevation:

    def __init__(self):
        self.azimuth   = 0.0 # radians
        self.elevation = 0.0 # radians

    def bodyCosinesToAngles(bodyCosines):
        bodyAngles = AzimuthElevation()

        bodyAngles.elevation = - (sin(bodyCosines.Z)
        bodyAngles.azimuth   = atan2(bodyCosines.Y, bodyCosines.X)

        return bodyAngles

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


class Attitude:
    
    def __init__(self):
        self.bank        = 0.0 # radians   +/- PI
        self.inclination = 0.0 # radians   +/- PI/2
        self.heading     = 0.0 # radians   0.. 2PI


class EarthPosition:

    def __init__(self):
        self.lat = 0.0 # radians
        self.lon = 0.0 # radians
        self.alt = 0.0 # metres


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


class NavPosition:

    def __init__(self):
        self.N = 0.0 # metres
        self.E = 0.0 # metres
        self.D = 0.0 # metres

    def get(earthPosition1, earthPosition2):
        pass

    def getNewPosition(navCosines, slantRange):
        pass

    def getNewPosition(initialPosition, velocity, acceleration, dt):
        pass

    def get(xyz1, xyz2):
        pass

class NavVelocity:

    def __init__(self):
        self.N = 0.0 # m/s
        self.E = 0.0 # m/s
        self.D = 0.0 # m/s

    def get(bodyVelocity, rotation):
        pass


class NavAcceleration:

    def __init__(self):
        self.N = 0.0 # m/s^^2
        self.E = 0.0 # m/s^^2
        self.D = 0.0 # m/s^^2

    def get(bodyAcceleration, rotationMatrix):
        pass

class NavCosines:

    def __init__(self):
        self.N = 0.0 
        self.E = 0.0 
        self.D = 0.0 

    def get(bodyCosines, rotationMatrix):
        pass

    def get(navAngles):
        pass


    def 

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
class RotationMatrix:

   def __init__(self):
      self.C_XX = 0.0 
      self.C_XY = 0.0 
      self.C_XZ = 0.0
      self.C_YX = 0.0 
      self.C_YY = 0.0 
      self.C_YZ = 0.0
      self.C_ZX = 0.0 
      self.C_ZY = 0.0 
      self.C_ZZ = 0.0

   def getNavToBodyRotation(attitude):
      pass

   def transposeMatrix(rotationMatrix):
      pass


class OmegaMatrix:

   def __init__(self):
      omega_11 = 0.0 
      omega_12 = 0.0 
      omega_13 = 0.0
      omega_21 = 0.0 
      omega_22 = 0.0 
      omega_23 = 0.0
      omega_31 = 0.0 
      omega_32 = 0.0 
      omega_33 = 0.0


   def getOmegaMatrix(latlong1, latlong2):
      pass


class XYZ:
    
    def __init__(self):
       self.X = 0.0 
       self.Y = 0.0 
       self.Z = 0.0 

    def getSlantRange(xyzStart, xyzEnd):
        pass

    def get(earthPosition):
        pass

    def get(navPosition, xyz):
        pass
     

def getImageRoll(attitude, camera):
    pass

def getSlantRange(navPosition):
    pass

