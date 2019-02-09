
#include "coordinates.cpp"
#include <tuple>
#include <Servo.h>
#include <map>
#include "navigation.cpp"

class lidar{

  std::map<coord_xy, int> obstacleMap;

  std::tuple<float,float>[] sweep_lidar(){
    //todo
  }

  void initialize_lidar(){
    Servo panLidar;
    Servo tiltLidar;
    //Attatch the lidar to a certain pin on the arduin so that it can communicate
    panLidar.attatch();
    tiltLidar.attatch();
  }

  coord_xy[] getObstacleMap(){
    return obstacleMap;
  }

  //Converst the radius distance coordinates into x y coordinates
  coord_xy convert_r_xy(float angle, float dist){
    coord_xy boat_location = bc.location
    float x = dist*cos(angle);
    float y = dist*sin(angle);
    return coord_xy({boat_location.x + x, boat_lcation.y + y})
  }

  void update_map(){
    //sweeps the lidar and returns a list of (angle, distance pairs)
    std::tuple<float,float>[] lidar_data = sweep_lidar();
    for(int i = 0; i < sizeOf(lidar_data); i++){
      //adds the angle of the lidar sensor with the angle of the boat's direction.
      angle_with_respect_to_boat = navigation::bc.boat_direction + std::get<0>lidar_data[i];
      //converts the polar coordiantes to xy coordinates we can use with our map.
      coord_xy obst = convert_r_xy(angle_with_respect_to_boat, std::get<1>lidar_data[i]);
      obstacleMap.insert(pair<coord_xy, int>(obst, obstacleMap[obst]+1))


    }
  }
}
=======
//#include "coordinates.cpp"
//#include <tuple>
//#include <Servo.h>
//#include <map>
//#include "navigation.cpp"
//
//class lidar{
//
//  std::map<coord_xy, int> obstacleMap;
//
//  std::tuple<float,float>[] sweep_lidar(){
//    //todo
//  }
//
//  void initialize_lidar(){
//    Servo panLidar;
//    Servo tiltLidar;
//    //Attatch the lidar to a certain pin on the arduino so that it can communicate
//    panLidar.attatch();
//    tiltLidar.attatch();
//  }
//
//  coord_xy getObstacleMap(){
//    return obstacleMap;
//  }
//
//  //Converst the radius distance coordinates into x y coordinates
//  coord_xy convert_r_xy(float angle, float dist){
//    coord_xy boat_location = bc.location;
//    float x = dist*cos(angle);
//    float y = dist*sin(angle);
//    return coord_xy({boat_location.x + x, boat_lcation.y + y});
//  }
//
//  void update_map(){
//    //sweeps the lidar and returns a list of (angle, distance pairs)
//    tuple[] lidar_data = sweep_lidar();
//    for(int i = 0; i < sizeOf(lidar_data); i++){
//      //adds the angle of the lidar sensor with the angle of the boat's direction.
//      angle_with_respect_to_boat = navigation::bc.boat_direction + std::get<0>lidar_data[i];
//      //converts the polar coordiantes to xy coordinates we can use with our map.
//      coord_xy obst = convert_r_xy(angle_with_respect_to_boat, std::get<1>lidar_data[i]);
//      obstacleMap.insert(pair<coord_xy, int>(obst, obstacleMap[obst]+1));
//
//
//    }
//  }
//}
>>>>>>> Stashed changes
