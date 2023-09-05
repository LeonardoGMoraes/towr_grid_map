
#include "towr_grid_map/grid2towr.h"
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
//#include <filters/filter_chain.hpp>

using namespace std;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;

  Grid2Towr myObject(n);
  myObject.GenericGridmap();
  myObject.ApplyFilter();

  std::vector<float> vector;
  vector = myObject.GetNormalVector(0.5,0.5);
  cout << "something  " << vector[0] << endl;
  cout << "something  " << vector[1] << endl;
  cout << "something  " << vector[2] << endl;



  ros::spin();

  return 0;
}



