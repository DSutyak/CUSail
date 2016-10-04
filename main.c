//
//  main.c
//  NavigationTests
//
//  Created by Alena Hutchinson on 10/1/16.
//  Copyright © 2016 Alena Hutchinson. All rights reserved.
//

#include <stdio.h>
#include "conv_wayPoints.h"

int main() {
    
    // coord_t data structure
    struct coord_t {
        double latitude;
        double longitude;
    };
    
    //test inputs for conv_wayPoints
    
    double wpTestArr[6] = {1.1, 1.2, 2.1, 2.2, 3.1, 3.2};
    int wpTestSize = 6;
    
//test conv_wayPoints function
   struct coord_t * converted_WPs = conv_wayPoints(wpTestArr, wpTestSize);
   printf("\ntype double waypoint array address is: %p\n", wpTestArr);
   printf("type coord_t waypoint array address is: %p\n", converted_WPs);

    printf("new address of wpTestArr is : %p\n", wpTestArr);

    return 0;
}
