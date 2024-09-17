#include "towr_grid_map/grid2towr.h"
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

/*
Grid2Towr::Grid2Towr(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle),
    filterChain_("grid_map::GridMap")
{

}

*/

Grid2Towr::Grid2Towr() 
{
  //this->RandomGridmap();
  //this->SlopesGridmap();
  //this->SlopesSinRandomGridmap();
  //this->RosbagGridmap();
  //this->Gaps();
  //this->Slope();
  this->SinGridmap();
  //this->Block01();
  //this->Block005();
  //this->Stairs();
  //this->Colision();

}

//Funcoes de geracao de gridmaps
void Grid2Towr::Slope()
{

  const double g1 = 0.45;
  const double g2 = 1.45;
  const double g3 = 2.45;
  const double g4 = 3.45;
  const double g5 = 4.45;

  const double a = 0.1;
  const double b = -a*g1;
  const double c = 0.25;
  const double d = (a-c)*g2 + b;
  const double e = 0.35;
  const double f = (c-e)*g3 + d;
  const double g = 0.45;
  const double h = (e-g)*g4 + f;
  const double i = 0.55;
  const double j = (g-i)*g5 + h;


  double hh;
  const double start = 1.0;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 3), 0.01);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        if (position.x()<start){
          hh = 0.0;
        }

        if ((position.x()>g1)&&(position.x()<=g2)){
          hh = a*position.x()+b;
        }
        if (position.x()>g2){
          hh = a*g2 + b +0.05;
        }
        map.at("elevation", *it) = hh;
    }
}

void Grid2Towr::Colision(){
  const double a = 0.09;
  const double l = 0.8;
  double hh;
  const double start = 1.0;
  const double end = 2.0;
  const double lat = 0.18;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        if (position.x()<start){
          hh = 0.0;
        }
        if ((position.x()>=start) && (position.x()<=end) && (position.y()<=lat) && (position.y()>=-lat)){
          hh = 0.5;
        }

        map.at("elevation", *it) = hh;
    }
}

void Grid2Towr::Stairs2()
{

  const double a = 0.06;
  const double l = 0.8;
  double hh;
  const double start = 1.0;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        if (position.x()<start){
          hh = 0.0;
        }
        if (position.x()>=start){
          hh = 1.0;
        }

        map.at("elevation", *it) = hh;
    }
}

void Grid2Towr::Stairs()
{

  const double a = 0.05;
  const double l = 0.8;
  double hh;
  const double start = 1.0;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 3), 0.01);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        if (position.x()<start){
          hh = 0.0;
        }

        if ((position.x()>=start) && (position.x()<(start + l))){
          hh = a;
        }

        if ((position.x()>=(start + l)) && (position.x() < (start + 2*l))){
          hh = 2*a;
        }

        if ((position.x()>=(start + 2*l)) &&  (position.x() < (start + 3*l))){
          hh = 3*a;
        }
        if ((position.x()>=(start + 3*l)) &&  (position.x() < (start + 4*l))){
          hh = 2*a;
        }
        if ((position.x()>=(start + 4*l)) &&  (position.x() < (start + 5*l))){
          hh = a;
        }
        if ((position.x()>=(start + 5*l))){
          hh = 0;
        }

        map.at("elevation", *it) = hh;
    }
}

void Grid2Towr::Gaps()
{

  const double a = 0.05;
  const double l = 0.8;
  double hh;
  const double start = 1.0;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 3), 0.02);

  

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        hh=0;

        if (position.x()<start){
          hh = 0.0;
        }

        if ((position.x()>=1.0) && (position.x()<(1.0 + 0.25))){
          hh = -1.0;
        }

        if ((position.x()>=(3.0)) &&  (position.x() < (3.0 + l))){
          hh = 0.15;
        }



        map.at("elevation", *it) = hh;
    }
}

void Grid2Towr::RosbagGridmap()
{

  /**/
  rosbag::Bag bag;
  bag.open("/home/leo/mestrado_ws/src/rosbags_mestrado/gridmap/terreno11_gridmap.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/elevation_mapping/elevation_map"));
  int i = 0;
  
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view){
    grid_map_msgs::GridMap::ConstPtr s = m.instantiate<grid_map_msgs::GridMap>();
    if (s != NULL)
      if (i==0){
        //std::cout << s->data[0].data[0] << std::endl;
        //std::cout << *s << std::endl;
        GridMapRosConverter::fromMessage(*s, map);
        
      }
      i++;

  }
  
  
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      if (std::to_string(map.at("elevation", *it)) == "nan"){ 
        map.at("elevation", *it) = -1.0; //Tratar nan
      }

      if (position.x() < 0.7){
        map.at("elevation", *it) = 0.0;
      }
      /*

      //Conferir os valores do Grid
      //if ((std::to_string((map.at("elevation", *it))) != "nan") && ((map.at("elevation", *it)) > 0.001)){
      if ((map.at("elevation", *it)) != 0){
        std::cout << "posicao em x: " << position.x();
        std::cout << " posicao em y: " << position.y();
        std::cout << " elevation: " << map.at("elevation", *it) << endl;
      }
      */

  }
  
  
  // Filter values for submap (iterators).
  map.add("elevation_filtered", map.get("elevation"));
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    double radius = 0.003; //tava 0.1 //acho que quanto menor, menos conta
    double mean = 0.0;
    double sumOfWeights = 0.0;
    // Compute weighted mean.
    for (CircleIterator circleIt(map, currentPosition, radius); !circleIt.isPastEnd(); ++circleIt) {
      if (!map.isValid(*circleIt, "elevation")) continue;
      Position currentPositionInCircle;
      map.getPosition(*circleIt, currentPositionInCircle);
      // Computed weighted mean based on Euclidian distance.
      double distance = (currentPosition - currentPositionInCircle).norm();
      double weight = pow(radius - distance, 2);
      mean += weight * map.at("elevation", *circleIt);
      sumOfWeights += weight;
    }
    map.at("elevation_filtered", *it) = mean / sumOfWeights;
  }
  
}

void Grid2Towr::RandomGridmap()
{

  const double lo = -0.03;
  const double hi = +0.03;

  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(8, 6), 0.2);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    //map.getLength().x(), map.getLength().y(),
    //map.getSize()(0), map.getSize()(1));

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        //map.at("elevation", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
        //map.at("elevation", *it) = -0.4 + 1.2 * std::sin(5.0 * position.y()) * position.x();
        //map.at("elevation", *it) = +0.00;
        //map.at("elevation", *it) = 0.01*position.x();
        //map.at("elevation", *it) = 0.2*std::sin(5.0 * position.x());
        float hh = lo + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(hi-lo)));
        //hh = frandom()
        //hh = 0.08;
        if (position.x()<=0.45){
          hh = 0.0;
        }
        //h = a_*sin(w_*x + pi/2);
        map.at("elevation", *it) = hh;
        //std::cout << map.at("elevation", *it) << endl;


      
    }
}

void Grid2Towr::SinGridmap()
{

  const double w_ = 6;
  const double a_ = 0.08;
  const double sin_start_ = 0.6;
  const double pi = 3.141592;
  double hh;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        hh = a_*sin(w_*position.x() + pi/2);

        if (position.x()<=0.45){
          hh = 0.0;
        }
        map.at("elevation", *it) = hh;
        //std::cout << "posicao em x: " << position.x();
        //std::cout << " posicao em y: " << position.y();
        //std::cout << " elevation: " << hh << endl;


      
    }
}

void Grid2Towr::Block01()
{

  double hh;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        hh = 0.0;

        if (position.x()>=0.75){
          hh = 0.1;
        }

        if (position.x()<=0.45){
          hh = 0.0;
        }
        map.at("elevation", *it) = hh;
        //std::cout << "posicao em x: " << position.x();
        //std::cout << " posicao em y: " << position.y();
        //std::cout << " elevation: " << hh << endl;


      
    }
}

void Grid2Towr::Block005()
{

  double hh;

  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);

        hh = 0.0;

        if (position.x()>=0.75){
          hh = 0.05;
        }

        if (position.x()<=0.45){
          hh = 0.0;
        }
        map.at("elevation", *it) = hh;
        //std::cout << "posicao em x: " << position.x();
        //std::cout << " posicao em y: " << position.y();
        //std::cout << " elevation: " << hh << endl;


      
    }
}

void Grid2Towr::SlopesGridmap()
{
  //y1 = ax + b
  //y2 = cx + d
  //y3 = ex + f
  //y4 = gx + h
  //y5 = ix + j

  double hh;
  const double g1 = 0.45;
  const double g2 = 1.45;
  const double g3 = 2.45;
  const double g4 = 3.45;
  const double g5 = 4.45;

  const double a = 0.15;
  const double b = -a*g1;
  const double c = 0.25;
  const double d = (a-c)*g2 + b;
  const double e = 0.35;
  const double f = (c-e)*g3 + d;
  const double g = 0.45;
  const double h = (e-g)*g4 + f;
  const double i = 0.55;
  const double j = (g-i)*g5 + h;


  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.05);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    //map.getLength().x(), map.getLength().y(),
    //map.getSize()(0), map.getSize()(1));

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);


        if (position.x()<=g1){
          hh = 0.0;
        }
        if ((position.x()>g1)&&(position.x()<=g2)){
          hh = a*position.x()+b;
        }
        if ((position.x()>g2)&&(position.x()<=g3)){
          hh = c*position.x()+d;
        }
        if ((position.x()>g3)&&(position.x()<=g4)){
          hh = e*position.x()+f;
        }
        if ((position.x()>g4)&&(position.x()<=g5)){
          hh = g*position.x()+h;
        }
        if (position.x()>g5){
          hh = i*position.x()+j;
        }
        map.at("elevation", *it) = hh;
        std::cout << "posicao em x: " << position.x();
        std::cout << " posicao em y: " << position.y();
        std::cout << " elevation: " << map.at("elevation", *it) << endl;


      
    }
}

void Grid2Towr::SlopesRandomGridmap(){
  const double lo = -0.03;
  const double hi = +0.03;
  float hh1 = 0.0;
  const double w_ = 6;
  const double a_ = 0.08;
  const double sin_start_ = 0.6;
  const double pi = 3.141592;
  double hh2;
  
  double hh3;
  const double g1 = 0.45;
  const double g2 = 1.45;
  const double g3 = 2.45;
  const double g4 = 3.45;
  const double g5 = 4.45;

  const double a = 0.15;
  //const double a = 0.0;
  const double b = -a*g1;
  const double c = 0.25;
  const double d = (a-c)*g2 + b;
  const double e = 0.45;
  const double f = (c-e)*g3 + d;
  const double g = 0.65;
  const double h = (e-g)*g4 + f;
  const double i = 0.85;
  const double j = (g-i)*g5 + h;
  double hhf;

  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.2);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    //map.getLength().x(), map.getLength().y(),
    //map.getSize()(0), map.getSize()(1));


    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        float hh1 = lo + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(hi-lo)));
        //hh2 = a_*sin(w_*position.x() + pi/2);
        hh2 = 0.0;
        if (position.x()<=g1){
          hhf = 0.0;
        }
        if ((position.x()>g1)&&(position.x()<=g2)){
          hh3 = a*position.x()+b;
          //hh3 = 0.0;
          hhf = hh1 + hh2 + hh3;
        }
        if ((position.x()>g2)&&(position.x()<=g3)){
          hh3 = c*position.x()+d;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if ((position.x()>g3)&&(position.x()<=g4)){
          hh3 = e*position.x()+f;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if ((position.x()>g4)&&(position.x()<=g5)){
          hh3 = g*position.x()+h;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if (position.x()>g5){
          hh3 = i*position.x()+j;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        map.at("elevation", *it) = hhf;
}
}

void Grid2Towr::SlopesSinRandomGridmap(){
  const double lo = -0.03;
  const double hi = +0.03;
  float hh1 = 0.0;
  const double w_ = 6;
  const double a_ = 0.08;
  const double sin_start_ = 0.6;
  const double pi = 3.141592;
  double hh2;
  
  double hh3;
  const double g1 = 0.45;
  const double g2 = 1.45;
  const double g3 = 2.45;
  const double g4 = 3.45;
  const double g5 = 4.45;

  const double a = 0.15;
  const double b = -a*g1;
  const double c = 0.25;
  const double d = (a-c)*g2 + b;
  const double e = 0.45;
  const double f = (c-e)*g3 + d;
  const double g = 0.65;
  const double h = (e-g)*g4 + f;
  const double i = 0.85;
  const double j = (g-i)*g5 + h;
  double hhf;

  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(10, 6), 0.2);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    //map.getLength().x(), map.getLength().y(),
    //map.getSize()(0), map.getSize()(1));


    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        float hh1 = lo + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(hi-lo)));
        hh2 = a_*sin(w_*position.x() + pi/2);
        //hh2 = 0.0;
        if (position.x()<=g1){
          hhf = 0.0;
        }
        if ((position.x()>g1)&&(position.x()<=g2)){
          hh3 = a*position.x()+b;
          //hh3 = 0.0;
          hhf = hh1 + hh2 + hh3;
        }
        if ((position.x()>g2)&&(position.x()<=g3)){
          hh3 = c*position.x()+d;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if ((position.x()>g3)&&(position.x()<=g4)){
          hh3 = e*position.x()+f;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if ((position.x()>g4)&&(position.x()<=g5)){
          hh3 = g*position.x()+h;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        if (position.x()>g5){
          hh3 = i*position.x()+j;
          hhf = hh1 + hh2 + hh3;
          //hh3 = 0.0;
        }
        map.at("elevation", *it) = hhf;
}
}

void Grid2Towr::SlopesGridmapTowr()
{

  double hh;
  const double slope_start_ = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;

  const double x_down_start_ = slope_start_+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center/up_length_;


  //Create a generic Gridmap
  map.add("elevation");
  map.setFrameId("map");
  map.setGeometry(Length(20, 15), 0.005);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    //map.getLength().x(), map.getLength().y(),
    //map.getSize()(0), map.getSize()(1));

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);


        if (position.x()<=slope_start_){
          hh = 0.0;
        }
        if ((position.x()>slope_start_)&&(position.x()<=x_down_start_)){
          hh = slope_*(position.x()-slope_start_);
        }
        if ((position.x()>x_down_start_)&&(position.x()<=x_flat_start_)){
          hh = height_center - slope_*(position.x()-x_down_start_);;
        }
        if (position.x()>x_flat_start_){
          hh = 0.0;
        }
        map.at("elevation", *it) = hh;


      
    }
}

//Get Height given x,y
double Grid2Towr::GetElevation(double x, double y)
{
  float felevation;
  try {
    felevation = map.atPosition("elevation",{x,y},InterpolationMethods::INTER_LINEAR);
  }
  catch(...){
    felevation = 0; //If a point is off the grid
  }
  double elevation = (double) felevation;

  return elevation; 
}

//Get Derivatives given x,y 
double Grid2Towr::GetHeightDerivWrtX2(double x, double y){
  double dx;
  dx = (-1*this->GetElevation(x-hd,y)+0*this->GetElevation(x,y)+1*this->GetElevation(x+hd,y))/(2*1.0*pow(hd,1));
  //numerical derivative

  if (fabs(dx)< 0.001){
    dx = 0.0; // avoid rounding errors
  }

  //if ((fabs(x) >= 2.6) && x <= 2.63 ){
  //  dx = 5/3; // avoid rounding errors
  //}

  return dx;
}

double Grid2Towr::GetHeightDerivWrtY2(double x, double y){
  double dy;
  dy = (-1*this->GetElevation(x,y-hd)+0*this->GetElevation(x,y)+1*this->GetElevation(x,y+hd))/(2*1.0*pow(hd,1));
  //dy = 0.0;

  if (fabs(dy)< 0.001){
    dy = 0.0;
  }
  return dy;
}

double Grid2Towr::GetHeightDerivWrtXX(double x, double y){
  double dxx;
  //dxx = (-1*this->GetHeightDerivWrtX2(x-0.03,y)+0*this->GetHeightDerivWrtX2(x,y)+1*this->GetHeightDerivWrtX2(x+0.03,y))/(2*1.0*pow(0.03,1));

  dxx = (1*this->GetElevation(x-hd,y)-2*this->GetElevation(x,y)+1*this->GetElevation(x+hd,y))/(1*1.0*pow(hd,2));
  //dxx = 0.0;

  if (fabs(dxx)< 0.001){
    dxx = 0.0;
  }

  return dxx;
}

double Grid2Towr::GetHeightDerivWrtYY(double x, double y){
  double dyy;
  //dyy = (-1*this->GetHeightDerivWrtY2(x,y-0.03)+0*this->GetHeightDerivWrtY2(x,y)+1*this->GetHeightDerivWrtY2(x,y+0.03))/(2*1.0*pow(0.03,1));

  dyy = (1*this->GetElevation(x,y-hd)-2*this->GetElevation(x,y)+1*this->GetElevation(x,y+hd))/(1*1.0*pow(hd,2));
  //dyy = 0.0;

  if (fabs(dyy)< 0.001){
    dyy = 0.0;
  }
  return dyy;
}

double Grid2Towr::GetHeightDerivWrtXY(double x, double y){
  double dxy;

  dxy = (-1*this->GetHeightDerivWrtX2(x,y-hd)+0*this->GetHeightDerivWrtX2(x,y)+1*this->GetHeightDerivWrtX2(x,y+hd))/(2*1.0*pow(hd,1));
  //dxy = 0.0;

  if (fabs(dxy)< 0.001){
    dxy = 0.0;
  }
  
  return dxy;
}

double Grid2Towr::GetHeightDerivWrtYX(double x, double y){
  double dyx;
  dyx = (-1*this->GetHeightDerivWrtY2(x-hd,y)+0*this->GetHeightDerivWrtY2(x,y)+1*this->GetHeightDerivWrtY2(x+hd,y))/(2*1.0*pow(hd,1));
  //dyx = 0.0;

  if (fabs(dyx)< 0.001){
    dyx = 0.0;
  }

  return dyx;
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






