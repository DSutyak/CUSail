//pseudocode for new sailing method

waypoint_vector=vector from waypoint to boat
normr=length of waypoint vector

if normr < 1 && waypoint_number <= size of waypoints:
	//reached a waypoint
	//should be modified for buoy rounding
	increment waypoint_number
	create new waypoint_vector
//angle to the waypoint from current position	
angle_waypoint=atan(r0, r1)#360/2PI
//angle to waypoint wrt wind
dirangle=angle_waypoint-windDir

//only needed for simulator because boatdir in simulator is CCW wrt x axis
boatDir=450-boatdir

// wind wrt boat
windboat=windDir-boatDir



if(dirangle<optpolartop){
	nextHeading=optpolartop;
	sailangle=windboat+15
	tailangle=windboat
	}
//we can sail directly to the target
//right side
else if(dirangle<(180-optpolarbot)){
	nextHeading=dirangle;

	sailangle=windboat+15
	tailangle=windboat
}
//we have to port jibe
//right down
else if(dirangle<(180)){
	nextHeading=180-optpolarbot;

	sailangle=windboat+15
	tailangle=windboat
}
//we have to starboard jibe
//left down
else if(dirangle<(180+optpolarbot)){
	nextHeading=180+optpolarbot;

	sailangle=windboat-15
	tailangle=windboat
}
//can sail directly to the target
//left side
else if(dirangle<(360-optpolartop)){
	nextHeading=dirangle;

	sailangle=windboat-15
	tailangle=windboat
}
//starboard tack
//left up
else{
	nextHeading=360-optpolartop;

	sailangle=windboat-15
	tailangle=windboat
}


#reached end of waypoints, execute infinity loop

