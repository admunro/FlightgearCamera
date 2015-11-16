#include "LDPMaths.hpp"

using namespace LDPMaths;

/*
 * Targets
 *
 */
const EarthPosition RUNWAY_25L_THRESHOLD(48.71408174 * DEG_2_RAD,
                                         11.55487973 * DEG_2_RAD,
                                         1196.8      * FT_2_M);

const EarthPosition RUNWAY_25L_MIDPOINT(48.70930059 * DEG_2_RAD,
                                        11.53684098 * DEG_2_RAD,
                                        1201.5 *  FT_2_M);

const EarthPosition RUNWAY_25R_THRESHOLD(48.72485816 * DEG_2_RAD,
                                         11.54611702 * DEG_2_RAD,
                                         1193.1      * FT_2_M);

const EarthPosition RUNWAY_07L_THRESHOLD(48.71668582 * DEG_2_RAD,
                                         11.51562345 * DEG_2_RAD,
                                         1192.4      * FT_2_M);

const EarthPosition RUNWAY_07R_THRESHOLD(48.70451943 * DEG_2_RAD,
                                         11.51880223 * DEG_2_RAD,
                                         1206.2      * FT_2_M);

const EarthPosition RUNWAY_25R_MIDPOINT(48.72077199  * DEG_2_RAD,
                                        11.530870235 * DEG_2_RAD,
                                        1192.75      * FT_2_M);

/*
 * Ownship positions
 *
 */
const EarthPosition SOUTH_OF_RWY_25L(48.5        * DEG_2_RAD,
                                     11.55487973 * DEG_2_RAD,
                                     10000       * FT_2_M);

const EarthPosition RUNWAY_25L_10K_APPROACH(48.746944 * DEG_2_RAD,
                                            11.681667 * DEG_2_RAD,
                                            7500      * FT_2_M);

const EarthPosition RUNWAY_25L_5K_APPROACH(48.730833 * DEG_2_RAD,
                                           11.618056 * DEG_2_RAD,
                                           5000      * FT_2_M);

const EarthPosition RUNWAY_07R_APPROACH(48.673611 * DEG_2_RAD,
                                        11.390833 * DEG_2_RAD,
                                        10000     * FT_2_M);


