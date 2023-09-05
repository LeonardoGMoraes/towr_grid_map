
#include "towr_grid_map/grid2towr.h"
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

Grid2Towr::Grid2Towr(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle),
    filterChain_("grid_map::GridMap")
{
  
}

void Grid2Towr::GenericGridmap()
{

  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("elevation", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
        //map.at("elevation", *it) = -0.04;
      
    }
}

float Grid2Towr::GetElevation(double x, double y)
{
  float elevation;
  elevation = map.atPosition("elevation",{x,y});

  return elevation; 
}

void Grid2Towr::ApplyFilter()
{
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  filterChain_.configure(filterChainParametersName_, nodeHandle_);
  filterChain_.update(map, outputMap);
}

std::vector<float> Grid2Towr::GetNormalVector(double x, double y)
{
  std::vector<float> normal;
  float n1;
  float n2;
  float n3;
  n1 = outputMap.atPosition("normal_vectors_x",{x,y});
  n2 = outputMap.atPosition("normal_vectors_y",{x,y});
  n3 = outputMap.atPosition("normal_vectors_z",{x,y});
  normal = {n1,n2,n3};

  return normal;
}

//void PlaneEquation(double x, double y)
//A plane containing the point (x1,y1,z1) and having a normal vector n = (a,b,c) is given by: a(x-x1)+ b(y-y1) + c(z-z1)=0

/*
double GetHeightDerivWrtX(double x, double y){

  Getnormalvector(double x, double y)

  return something
}

double GetHeightDerivWrtY(double x, double y){

  Getnormalvector(double x, double y)

  return something
}

double GetHeightDerivWrtXX(double x, double y){

  Getnormalvector(double x, double y)

  return something
}

double GetHeightDerivWrtYY(double x, double y){

  Getnormalvector(double x, double y)

  return something
}

double GetHeightDerivWrtXY(double x, double y){

  Getnormalvector(double x, double y)

  return something
}

double GetHeightDerivWrtYX(double x, double y){

  Getnormalvector(double x, double y)

  return something
}
*/






