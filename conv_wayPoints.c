//
//  conv_wayPoints.c
//  NavigationTests
//
//  Created by Alena Hutchinson on 10/1/16.
//  Copyright © 2016 Alena Hutchinson. All rights reserved.
//

#include "conv_wayPoints.h"

struct coord_t {
    double latitude;
    double longitude;
};


// function to convert wayPoints double array into array of type coord_t
struct coord_t * conv_wayPoints (double *wayPoints, float waypointsize) {
    
    struct coord_t converted_wayPoints[((int)waypointsize/2)];
    
    for (int i = 0; i < waypointsize/2; i++) {
    
        printf("i is : %i\n\n", i);

        converted_wayPoints[i].latitude = *wayPoints;
        
        printf("latitude is: %.1f\n", *wayPoints);
        printf("converted_waypoints[%i].latitude is %.1f\n\n: ", i, converted_wayPoints[i].latitude);

        wayPoints = wayPoints + 1;
        converted_wayPoints[i].longitude = *wayPoints;
        
        printf("longitude is:  %.1f\n", *wayPoints);
        printf("converted_waypoints[%d].longitude is %.1f\n\n: ", i, converted_wayPoints[i].longitude);
        
        wayPoints = wayPoints + 1;

    }
    
    printf("type coord_t wayPoints : ");
    for (int i = 0; i < waypointsize/2; i++) {
        printf("[%.2f, %.2f] ,", converted_wayPoints[i].latitude, converted_wayPoints[i].longitude);
    }

    return converted_wayPoints;
    
}
