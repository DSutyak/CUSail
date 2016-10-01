//
//  conv_wayPoints.c
//  NavigationTests
//
//  Created by Alena Hutchinson on 10/1/16.
//  Copyright © 2016 Alena Hutchinson. All rights reserved.
//

#include "conv_wayPoints.h"

// varibale declaration
double *wayPoints;
int waypointsize;

struct coord_t {
    double latitude;
    double longitude;
};

struct coord_t *converted_wayPoints;

// function to convert wayPoints double array into array of type coord_t
struct coord_t * conv_wayPoints (double *wayPoints, int waypointsize) {
    
    int i, j = 0;
    struct coord_t *converted_wayPoints[(waypointsize/2)];
    
    
    for ( j = 0; j < waypointsize - 1; j = j+2) {
        converted_wayPoints[i]->latitude = wayPoints[j];
        converted_wayPoints[i]->longitude = wayPoints[(j+1)];
        i++;
    }
    
    return *converted_wayPoints;
}