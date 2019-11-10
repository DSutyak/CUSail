#include "TinyGPS++.h"
TinyGPSPlus gps;

double latOffset;
double longOffset;
double longScale;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters

#ifndef coordinate_h
#define coordinate_h

typedef struct coord_t {
  double latitude; // float latitude
  double longitude; // float longitudes
}coord_t;
#endif

#ifndef coord_xy_h
#define coord_xy_h
typedef struct _coord_xy {
  double x; // float x coord
  double y; // float y coord
} coord_xy;
#endif

coord_xy origin;

/*Converts coordinate in latitude and longitude to xy*/
coord_xy xyPoint(coord_t latlong){
  double x = (latlong.longitude - longOffset) * longScale * latToMeter;
  double y = (latlong.latitude - latOffset) * latToMeter;
  return coord_xy({x, y});
}

void setOrigin(coord_t startPoint){
  origin = coord_xy({(double) 0, (double) 0});
  longOffset = startPoint.longitude; //used to generate X coordinate
  latOffset = startPoint.latitude; //used to generate Y coodinate
  longScale = cos(latOffset * M_PI/180);  //scaling factor to account for changing distance between longitude lines
}

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial3.begin(9600);
  Serial1.print("Beginning Setup");
  gps.encode(Serial3.read());
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  coord_t latlong = {lat,lon};
  setOrigin(latlong);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial1.print("At Loop\n");
  while (Serial3.available() > 0) {
      gps.encode(Serial3.read());
      coord_t coord_lat_lon = {gps.location.lat(), gps.location.lng()};
      coord_xy currentPosition = xyPoint(coord_lat_lon);
      Serial1.print("----------NAVIGATION----------");
      Serial1.print("\n");
      Serial1.print("X position: "); 
      Serial1.println(currentPosition.x,10);
      Serial1.print("\n");
      Serial1.print("Y position: "); 
      Serial1.println(currentPosition.y,10);
      Serial1.print("\n");
//      Serial1.print("Latitude:");
//      Serial1.print(gps.location.lat(),10);
//      Serial1.print("\n");
//      Serial1.print("Longitude:");
//      Serial1.print(gps.location.lng(),10);
//      Serial1.print("\n");
      Serial1.print("----------END----------");
      Serial1.print("\n");
  }
}
