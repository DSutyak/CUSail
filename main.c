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
    
    //test inputs for conv_wayPoints
    double wpTestArr[6] = {1.1, 1.2, 2.1, 2.2, 3.1, 3.2};
    int wpTestSize = 6;
    
    //test conv_wayPoints function
    struct coord_t * converted_WPs = conv_wayPoints(wpTestArr, wpTestSize);
    printf("type double waypoint array is: %p\n", wpTestArr);
    printf("type coord_t waypoint array is: %p\n", converted_WPs);

    return 0;
}
