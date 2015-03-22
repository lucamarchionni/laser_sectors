#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <vector>

struct Sector
{
  double angle_min;
  double angle_max;
  double range;
};

typedef std::vector<Sector> LaserSectors;

class LaserProcessor
{
public:
  LaserProcessor();

  void laserCB(sensor_msgs::LaserScan const &laser);

  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher  sector_pub_;
};


LaserProcessor::LaserProcessor()
{
  laser_sub_  = nh_.subscribe("/scan", 1, &LaserProcessor::laserCB,this);
  sector_pub_ = nh_.advertise<visualization_msgs::Marker>("sectors",10);
}

void LaserProcessor::laserCB(sensor_msgs::LaserScan const &laser)
{
  if(laser.ranges.size() < 1)
    return;
  LaserSectors  sectors;

  Sector sector;
  sector.angle_min = laser.angle_min;
  sector.angle_max = sector.angle_min;
  sector.range = laser.ranges.at(0);
  sectors.push_back(sector);
  int index = 0;
  for(unsigned int i=1; i< laser.ranges.size(); ++i)
  {
    if( fabs(laser.ranges.at(i) - sectors.at(index).range) < 0.05 )
    {
      sectors[index].angle_max += laser.angle_increment;
    }
    else
    {
      sector.angle_min = sectors.at(index).angle_max + laser.angle_increment;
      sector.angle_max = sector.angle_min;
      sector.range = laser.ranges.at(i);
      sectors.push_back(sector);
      index++;
    }
  }

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  std_msgs::ColorRGBA color;
  color.r = 1.0;

  marker.header = laser.header;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  for(unsigned int i=0; i< sectors.size(); ++i)
  {
    marker.colors.push_back(color);
    geometry_msgs::Point point1,point2, point3;
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;

    point2.x = sectors[i].range*cos(sectors[i].angle_min);
    point2.y = sectors[i].range*sin(sectors[i].angle_min);
    point2.z = 0;

    point3.x = sectors[i].range*cos(sectors[i].angle_max);
    point3.y = sectors[i].range*sin(sectors[i].angle_max);
    point3.z = 0;


    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.points.push_back(point3);
  }

  sector_pub_.publish(marker);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "laser_processor");

  LaserProcessor lp;

  ros::spin();

  return 0;
}
