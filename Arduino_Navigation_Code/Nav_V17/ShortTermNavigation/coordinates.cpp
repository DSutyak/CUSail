#include "navigation_helper.h"
#include "navigation.h"

//Coordinates in and around the Engineering Quad, Cornell university
coord_t olin = {42.445457, -76.484322}; //Olin Hall
coord_t hollister = {42.444259, -76.484435}; //Hollister Hall
coord_t outsideDuffield = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
coord_t outsideThurston = {42.444228, -76.483666}; //In front of Thurston Hall
coord_t engQuadX = {42.444612, -76.483492}; //Center of Engineering Quad
coord_t northBottomStairs = {42.444735, -76.483275}; //Bottom of stairs, to the left of North entrance to Duffield Hall
coord_t engQuadRight = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad
coord_t sundial = {42.4448630, -76.4835293}; //Sundial on the engineering quad
coord_t bottomRightIntersection= {42.444242, -76.483247}; //path intersection bottom right of quad
coord_t thurstonWestEntrance={42.444245, -76.484068}; //west entrance to thurston on the path
coord_t ctb{42.442539, -76.485019};
coord_t stairsToCTown{42.444057, -76.484228}; //top of stairs between bard and hollister

//Coordinates in and around the Cornell Sailing Center
coord_t lakeOut = {42.469386,-76.504690}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut4 = {42.469847,-76.504522}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut_beach_far = {42.469065,-76.506674}; //Out in the lake, to the right of the Cornell Sailing Center
coord_t lakeOut_beach = {42.470894,-76.504712}; //Out in the lake, to the right of the Cornell Sailing Center but further North
coord_t lakeOut_beach2 = {42.470535, -76.50489};
coord_t lakeOut_beach3 = {42.471124, -76.505757};
coord_t shore_beach = {42.470862,-76.503950}; //Beach, to the right of the Cornell Sailing Center
coord_t acrossDock = {42.465702, -76.524625}; //Across the lake, when looking from the far edge of the dock to the right of the Cornell Sailing Center
coord_t acrossBeach = {42.467918, -76.525419}; //Across the lake, when looking from the beach to the left of the Cornell Sailing Center
coord_t low_dock={42.468951,-76.502941}; //Right at the lower dock
coord_t high_dock={42.469552,-76.503353}; //High dock launch point

coord_t dirt_mound={42.454386, -76.475693};
coord_t appel_door={42.453915, -76.475891};
coord_t appel_farRight={42.454345, -76.474471};
coord_t appel_topRight={42.454715, -76.474551};
coord_t appel_topLeft={42.454865, -76.475672};
coord_t appel_bottomRight={42.453828, -76.474454};

// COORDINATES AT US NAVAL ACADEMY
coord_t end_0 = {38.982585,-76.477650};
coord_t end_1 = {38.982893, -76.477089};
coord_t end_2 = {38.982273, -76.476141};
coord_t end_3 = {38.982130, -76.475690};
coord_t end_4 = {38.982436, -76.475389};
coord_t end_5 = {38.982820, -76.475041};
coord_t end_6 = {38.983139, -76.474748};
coord_t end_7 = {38.983570, -76.474500};
coord_t end_8 = {38.984001, -76.475111};
coord_t end_9 = {38.984429, -76.475765};
coord_t end_10 = {38.984887, -76.476336};
coord_t end_11 = {38.985242, -76.476844};
coord_t end_12 = {38.985730, -76.477460};
coord_t end_13 = {38.985274, -76.477820};
coord_t end_14 = {38.984842, -76.478194};
coord_t end_15 = {38.984389, -76.478585};
coord_t end_16 = {38.984030, -76.479000};
coord_t end_17 = {38.983646, -76.478501};
coord_t end_18 = {38.983304, -76.478074};
coord_t end_19 = {38.982930, -76.477625};