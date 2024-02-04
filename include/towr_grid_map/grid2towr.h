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

  GridMap map;
  GridMap outputMap;
  //filters::FilterChain<grid_map::GridMap> filterChain_;
  //std::string filterChainParametersName_;
  //ros::NodeHandle& nodeHandle_;

  //Grid2Towr(ros::NodeHandle& nodeHandle);
  Grid2Towr();
  
  //Funcoes Para Geracao de Grids
  void RandomGridmap();
  void SinGridmap();
  void SlopesGridmap();
  void SlopesRandomGridmap();
  void SlopesSinRandomGridmap();
  void SlopesGridmapTowr();

  
  //altura no ponto (x,y)
  //dado um ponto x,y retornar a altura
  double GetElevation(double x, double y);
  
  //aplicar os filtros no grid obtido
  void ApplyFilter();

  //dado um ponto x,y retornar a normal (retornar um vector3D)
  std::vector<double> GetNormalVector(double x, double y);


  //Derivada aplicada naquele ponto
  //Calculado pelas componentes do vetor normal com a componente z=1
  double GetHeightDerivWrtX1(double x, double y);
  double GetHeightDerivWrtY1(double x, double y);

  //Derivada aplicada naquele ponto
  //Calculado numericamente
  double GetHeightDerivWrtX2(double x, double y);
  double GetHeightDerivWrtY2(double x, double y);

  //Derivada segunda aplicada naquele ponto
  double GetHeightDerivWrtXX(double x, double y);
  double GetHeightDerivWrtYY(double x, double y);
  double GetHeightDerivWrtXY(double x, double y);
  double GetHeightDerivWrtYX(double x, double y);


};

#endif