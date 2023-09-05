#ifndef GRID2TOWR_H
#define GRID2TOWR_H

#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <filters/filter_chain.hpp>

using namespace std;
using namespace grid_map;


class Grid2Towr
{
public:

  int a;
  //grid_map::GridMap map;
  GridMap map;
  //grid_map::GridMap outputMap;
  GridMap outputMap;
  filters::FilterChain<grid_map::GridMap> filterChain_;
  std::string filterChainParametersName_;
  ros::NodeHandle& nodeHandle_;

  Grid2Towr(ros::NodeHandle& nodeHandle);
  
  //obter o grid e.g msg em ROS
  //Converter uma msg (em um determinado topico) para um gridmap
  void GenericGridmap();

  //altura no ponto (x,y)
  //dado um ponto x,y retornar a altura
  float GetElevation(double x, double y);
  
  //aplicar os filtros no grid obtido
  void ApplyFilter();

  //dado um ponto x,y retornar a normal (retornar um vector3D)
  std::vector<float> GetNormalVector(double x, double y);

  //Equacao do Plano no ponto x y
  void PlaneEquation(double x, double y);


  //Derivada aplicada naquele ponto
  double GetHeightDerivWrtX(double x, double y);
  double GetHeightDerivWrtY(double x, double y);

  //Derivada segunda aplicada naquele ponto
  double GetHeightDerivWrtXX(double x, double y);
  double GetHeightDerivWrtYY(double x, double y);
  double GetHeightDerivWrtXY(double x, double y);
  double GetHeightDerivWrtYX(double x, double y);

  //vetor normal ao plano no ponto (x,y)

 

};

#endif