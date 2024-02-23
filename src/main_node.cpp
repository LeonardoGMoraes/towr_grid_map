
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

  //Grid2Towr myObject(n);
  //myObject.GenericGridmap();
  //myObject.ApplyFilter();

  Grid2Towr myObject;
  //myObject.GenericGridmap();

  double px = 1.5;
  double py = 0.0;
  
  
  //double altura = myObject.GetElevation(px,py);
  //double altura = myObject.GetElevation(px,py);
  //cout << "altura:   " << altura << endl;
  
  
  /*
  std::vector<double> vector;
  vector = myObject.GetNormalVector(px,py);
  cout << "comp1  " << vector[0] << endl;
  cout << "comp2  " << vector[1] << endl;
  cout << "comp3  " << vector[2] << endl;
  

  double dx;
  dx = myObject.GetHeightDerivWrtX1(px,py);
  cout << "derivada em x (pelas componentes do vetor normal com z = 1):  " << dx << endl;
  */

  double dx2;
  dx2 = myObject.GetHeightDerivWrtX2(px,py);
  //cout << "derivada em x (numericamente): " << dx2 << endl;

  double dxx;
  dxx = myObject.GetHeightDerivWrtXX(px,py);
  //cout << "derivada segunda em x (numericamente): " << dxx << endl;

  double dxy;
  dxy = myObject.GetHeightDerivWrtXY(px,py);
  //cout << "derivada segunda em xy (numericamente): " << dxy << endl;

  double dyx;
  dyx = myObject.GetHeightDerivWrtYX(px,py);
  //cout << "derivada segunda em yx (numericamente): " << dyx << endl;

/*
  for (int i=0; i<40; i++){
    vector = myObject.GetNormalVector((0.1+i/10),(0.1+i/10));
    cout << "something  " << vector[2] << endl;

}

*/



  ros::spin();

  return 0;
}



