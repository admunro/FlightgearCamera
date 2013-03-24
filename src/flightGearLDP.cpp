#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
//#include <chrono>
//#include <ctime>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "LDPMaths.hpp"
#include "Tracking.hpp"
#include "Waypoints.hpp"

using namespace std;

    // Field of view values for the various sensor settings
    const double FOV_SW_IR = 24.0  * DEG_2_RAD;
    const double FOV_W_IR  =  2.8  * DEG_2_RAD;
    const double FOV_N_IR  =  0.77 * DEG_2_RAD;

    // Switch range - threshold for fiddling with the eyepoint
    const double SWITCH_RANGE = 50000.0;

    const AzimuthElevation NO_SLEW        (0.0, 0.0);
    const AzimuthElevation AZIMUTH_SLEW   (0.1, 0.0);
    const AzimuthElevation ELEVATION_SLEW (0.0, 0.1);
    const AzimuthElevation BOTH_SLEW      (0.1, 0.1);




    EarthPosition    ownship;
    Attitude         attitude;

    EarthPosition    target;

    CameraData       ldpCamera; 


    const bool MOVE_OWNSHIP = true;



    const double deltaTime = 0.02;


void die(string s)
{
   perror(s.c_str());
   exit(1);
}

int main()
{
   const int BUFLEN          = 256;
   const int PORT            = 8888;
   const std::string SERVER  = "localhost";
  
   // Set up network stuff
   struct sockaddr_in si_other;
   int s, slen = sizeof(si_other);

   char message[BUFLEN];

   // Open the connection
   if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
   {
      die("Couldn't open socket");
   }

   memset((char*) &si_other, 0, sizeof(si_other));
   si_other.sin_family = AF_INET;
   si_other.sin_port   = htons(PORT);

   if (inet_aton(SERVER.c_str(), &si_other.sin_addr) == 0)
   {
      std::cout << "inet_aton() failed." << std::endl;
   }


   
   cout << setprecision(10);


   attitude.bank        = 0.0;
   attitude.inclination = 0.0;
   attitude.heading     = 0.0;
 
   ownship = SOUTH_OF_RWY_25L;
   target  = RUNWAY_25R_MIDPOINT;
   
   ldpCamera.starePoint = target;

   for (int i = 1; i <= 425; i++)
   {

	  AzimuthElevation slew;

	  if (MOVE_OWNSHIP)
	  {
         ownship.lat += 0.00001;
         //ownship.lon += 0.00001;
	  }

	  if (i < 100 || i > 200)
	  {
	     slew = NO_SLEW;
	  }
	  else if (i < 200)
	  {
         slew = NO_SLEW;
	  }

      ldpCamera = lookAtLatLongAlt(ldpCamera.starePoint,
                                   ownship,
                                   attitude,
                                   FOV_SW_IR,
                                   slew,
                                   deltaTime);

      cout << "Camera starepoint Lat: " << ldpCamera.starePoint.lat * RAD_2_DEG
    	   << " Lon: " << ldpCamera.starePoint.lon * RAD_2_DEG
    	   << " Alt: " << ldpCamera.starePoint.alt * M_2_FT
    	   << endl;

//      cout << "FoV: " << ldpCamera.FoV * RAD_2_DEG
//
//           << " eyepoint - Lat: " << ldpCamera.eyePoint.lat * RAD_2_DEG
//           << " deg  Lon: "       << ldpCamera.eyePoint.lon * RAD_2_DEG
//           << " deg  Alt: "       << ldpCamera.eyePoint.alt
//           << " m  azimuth: "     << ldpCamera.anglesNav.azimuth   * RAD_2_DEG
//           << " deg  elevation: " << ldpCamera.anglesNav.elevation * RAD_2_DEG
//
//           << endl;

      sprintf(message, "%f, %f, %f, 0.0, %f, %f, %f\n",
              ldpCamera.eyePoint.lat * RAD_2_DEG,
              ldpCamera.eyePoint.lon * RAD_2_DEG,
              ldpCamera.eyePoint.alt * M_2_FT,
              ldpCamera.anglesNav.elevation * RAD_2_DEG,
              ldpCamera.anglesNav.azimuth * RAD_2_DEG,
              ldpCamera.FoV * RAD_2_DEG);

      if (sendto(s, message, strlen(message), 0, 
                 (struct sockaddr*) & si_other, slen) == 1)
      {
         die("Sendto.");
      }

      usleep(40000.0);

   }

   return 0;
}

