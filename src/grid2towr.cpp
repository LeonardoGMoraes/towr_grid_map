
#include "towr_grid_map/grid2towr.h"
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>


/*
Grid2Towr::Grid2Towr(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle),
    filterChain_("grid_map::GridMap")
{

}

*/

Grid2Towr::Grid2Towr() 
{
  
}


void Grid2Towr::GenericGridmap()
{

  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(2, 2), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("elevation", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
        //map.at("elevation", *it) = -0.4 + 1.2 * std::sin(5.0 * position.y()) * position.x();
        //map.at("elevation", *it) = -0.04;
        //map.at("elevation", *it) = 0.2*std::sin(5.0 * position.x());


      
    }
}

double Grid2Towr::GetElevation(double x, double y)
{
  float felevation;

  try {
    felevation = map.atPosition("elevation",{x,y},InterpolationMethods::INTER_NEAREST);
  }

  catch(...){
    felevation = 0;
  }
  
  double elevation = (double) felevation;

  return elevation; 
}

/*
void Grid2Towr::ApplyFilter()
  
{
  //ros::init(argc,argv, "main_node2");
  //ros::NodeHandle& nodeHandle_;
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  filterChain_.configure(filterChainParametersName_, nodeHandle_);
  filterChain_.update(map, outputMap);
  //ros::spin();
}
*/
/*
std::vector<double> Grid2Towr::GetNormalVector(double x, double y)
{
  //A plane containing the point (x1,y1,z1) and having a normal vector n = (a,b,c) is given by: a(x-x1)+ b(y-y1) + c(z-z1)=0
  std::vector<double> normal;
  float n1f;
  float n2f;
  float n3f;

  try{
    n1f = outputMap.atPosition("normal_vectors_x",{x,y},InterpolationMethods::INTER_NEAREST);
    n2f = outputMap.atPosition("normal_vectors_y",{x,y},InterpolationMethods::INTER_NEAREST);
    n3f = outputMap.atPosition("normal_vectors_z",{x,y},InterpolationMethods::INTER_NEAREST);
  }

  catch(...){
    n1f = 0;
    n2f = 0;
    n3f = 0;
  }
  double n1 = (double) n1f;
  double n2 = (double) n2f;
  double n3 = (double) n3f;

  normal = {n1,n2,n3};

  return normal;
}

double Grid2Towr::GetHeightDerivWrtX1(double x, double y){

  std::vector<double> vector;
  vector = this->GetNormalVector(x,y);
  std::vector<double> vector2;
  double dx;

  if (vector[2] != 0) {
    vector2 = {vector[0]/vector[2],vector[1]/vector[2],vector[2]/vector[2]};
    dx = -vector2[0];
  }

  else{
    vector2 = {0,0,0};
    dx = -vector2[0];
  }
  return dx;
}

double Grid2Towr::GetHeightDerivWrtY1(double x, double y){

  std::vector<double> vector;
  vector = this->GetNormalVector(x,y);
  std::vector<double> vector2;
  double dy;

  if (vector[2] != 0) {
    vector2 = {vector[0]/vector[2],vector[1]/vector[2],vector[2]/vector[2]};
    dy = -vector2[1];
  }

  else{
    vector2 = {0,0,0};
    dy = -vector2[1];
  }


  return dy;
}
*/

double Grid2Towr::GetHeightDerivWrtX2(double x, double y){
  double dx;
  dx = (-1*this->GetElevation(x-0.03,y)+0*this->GetElevation(x,y)+1*this->GetElevation(x+0.03,y))/(2*1.0*pow(0.03,1));
  return dx;
}

double Grid2Towr::GetHeightDerivWrtY2(double x, double y){
  double dy;
  dy = (-1*this->GetElevation(x,y-0.03)+0*this->GetElevation(x,y)+1*this->GetElevation(x,y+0.03))/(2*1.0*pow(0.03,1));
  return dy;
}

double Grid2Towr::GetHeightDerivWrtXX(double x, double y){
  double dxx;
  //dxx = (-1*this->GetHeightDerivWrtX2(x-0.03,y)+0*this->GetHeightDerivWrtX2(x,y)+1*this->GetHeightDerivWrtX2(x+0.03,y))/(2*1.0*pow(0.03,1));
  dxx = (1*this->GetElevation(x-0.03,y)-2*this->GetElevation(x,y)+1*this->GetElevation(x+0.03,y))/(1*1.0*pow(0.03,2));

  return dxx;
}

double Grid2Towr::GetHeightDerivWrtYY(double x, double y){
  double dyy;
  //dyy = (-1*this->GetHeightDerivWrtY2(x,y-0.03)+0*this->GetHeightDerivWrtY2(x,y)+1*this->GetHeightDerivWrtY2(x,y+0.03))/(2*1.0*pow(0.03,1));
  dyy = (1*this->GetElevation(x,y-0.03)-2*this->GetElevation(x,y)+1*this->GetElevation(x,y+0.03))/(1*1.0*pow(0.03,2));

  return dyy;
}

double Grid2Towr::GetHeightDerivWrtXY(double x, double y){
  double dxy;
  dxy = (-1*this->GetHeightDerivWrtX2(x,y-0.03)+0*this->GetHeightDerivWrtX2(x,y)+1*this->GetHeightDerivWrtX2(x,y+0.03))/(2*1.0*pow(0.03,1));
  
  return dxy;
}

double Grid2Towr::GetHeightDerivWrtYX(double x, double y){
  double dyx;
  dyx = (-1*this->GetHeightDerivWrtY2(x-0.03,y)+0*this->GetHeightDerivWrtY2(x,y)+1*this->GetHeightDerivWrtY2(x+0.03,y))/(2*1.0*pow(0.03,1));

  return dyx;
}







