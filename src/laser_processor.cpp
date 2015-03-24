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

struct Cluster
{
  std::vector<float> ranges;
  std::vector<float> angles;
  int id;
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
  std::vector<float> ranges = laser.ranges;
   for(unsigned int i=0; i< ranges.size(); ++i)
  {
    if(ranges.at(i) >  laser.range_max || ranges.at(i) < laser.range_min )
      ranges[i] = laser.range_max;
  }

  if(ranges.empty())
    return;

  std::vector<int> range_clusters(ranges.size(), 0);
  std::vector<Cluster> clusters;
  Cluster cluster;
  cluster.ranges.push_back(ranges.at(0));
  cluster.angles.push_back(laser.angle_min);
  cluster.id = 0;
  clusters.push_back(cluster);
  for(unsigned int i=1; i< ranges.size(); ++i)
  {
    double x1 = ranges.at(i-1) * cos(laser.angle_min + (i-1)*laser.angle_increment);
    double y1 = ranges.at(i-1) * sin(laser.angle_min + (i-1)*laser.angle_increment);

    double x2 = ranges.at(i) * cos(laser.angle_min + i*laser.angle_increment);
    double y2 = ranges.at(i) * sin(laser.angle_min + i*laser.angle_increment);

    if( sqrt( pow((x1-x2), 2) + pow( (y1-y2), 2)) < (0.4*ranges[i]/laser.range_max) ) //(4.0*ranges.at(i-1)*sin(2.0*laser.angle_increment)))
    {
      range_clusters[i] = range_clusters.at(i-1);
      clusters.back().ranges.push_back(ranges.at(i));
      clusters.back().angles.push_back( (laser.angle_min + i*laser.angle_increment));
    }
    else
    {
      range_clusters[i] = range_clusters.at(i-1)+1;
      Cluster cluster;
      cluster.ranges.push_back(ranges.at(i));
      cluster.angles.push_back((laser.angle_min + i*laser.angle_increment));
      cluster.id = clusters.back().id + 1;
      clusters.push_back(cluster);
    }
  }

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  
  marker.ns = "my_namespace";
  marker.id = 0;

  marker.action = visualization_msgs::Marker::ADD;

  marker.header = laser.header;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  int nclusters = range_clusters.at( range_clusters.size() - 1 );
  for(unsigned int i=0; i< range_clusters.size(); ++i)
  {
    std_msgs::ColorRGBA color;
    color.r = 1.0 - ( (double) (range_clusters.at(i)%10) / 9 );
    color.g = ( (double) (range_clusters.at(i)%10) / 9 );
    color.b = ( (double) (range_clusters.at(i)%10) / 9 );

    color.a = 1.0;
    marker.colors.push_back(color);

    geometry_msgs::Point point1,point2, point3;
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;

    point2.x = ranges[i]*cos(laser.angle_min + i*laser.angle_increment);
    point2.y = ranges[i]*sin(laser.angle_min + i*laser.angle_increment);
    point2.z = 0;

    point3.x = ranges[i]*cos(laser.angle_min + (i+1)*laser.angle_increment);
    point3.y = ranges[i]*sin(laser.angle_min + (i+1)*laser.angle_increment);
    point3.z = 0;


    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.points.push_back(point3);
  }

  
  for(unsigned int i=0; i < clusters.size() ; ++i)
  {
   
    if(clusters.size() < 2)
      break;
 
    if ( (i==0 && clusters.at(i).ranges.back() < clusters.at(i+1).ranges.front()) ||
         (i == (clusters.size() - 1) && clusters.at(i).ranges.front() > clusters.at(i-1).ranges.back() ) ||
         (i>0 && i<(clusters.size()-1) && 
          (clusters.at(i-1).ranges.back() < clusters.at(i).ranges.front() || 
          clusters.at(i).ranges.back() > clusters.at(i+1).ranges.front() )))     {
      
      double x1 = clusters.at(i).ranges.front()*cos(clusters.at(i).angles.front());
      double y1 = clusters.at(i).ranges.front()*sin(clusters.at(i).angles.front());
      double x2 = clusters.at(i).ranges.back()*cos(clusters.at(i).angles.back());
      double y2 = clusters.at(i).ranges.back()*sin(clusters.at(i).angles.back());

      double cluster_width = sqrt( pow(x1-x2,2) + pow(y1-y2,2));
      //if( cluster_width < 0.5)
      //  continue;

      ROS_ERROR_STREAM ("Possible corridor between angle " << clusters.at(i).angles.front() << " and " << clusters.at(i).angles.back());
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;

      color.a = 1.0;
      marker.colors.push_back(color);

    geometry_msgs::Point point1,point2, point3;
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;

    point2.x = clusters.at(i).ranges.front()*cos(clusters.at(i).angles.front());
    point2.y = clusters.at(i).ranges.front()*sin(clusters.at(i).angles.front());
    point2.z = 0;

    point3.x = clusters.at(i).ranges.back()*cos(clusters.at(i).angles.back());
    point3.y = clusters.at(i).ranges.back()*sin(clusters.at(i).angles.back());
    point3.z = 0;


    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.points.push_back(point3);
    }
  }

  sector_pub_.publish(marker);

 /* LaserSectors  sectors; 
  Sector sector;
  sector.angle_min = laser.angle_min;
  sector.angle_max = sector.angle_min;
  sector.range = laser.ranges.at(0);
  sector.range = sector.range > laser.range_max ? laser.range_max : sector.range;
  sector.range = sector.range < laser.range_min ? laser.range_max : sector.range;
  sectors.push_back(sector);
  int index = 0;
  for(unsigned int i=1; i< laser.ranges.size(); ++i)
  {
    if(laser.ranges.at(i) > laser.range_max ||
       laser.ranges.at(i) < laser.range_min || 
       (fabs(laser.ranges.at(i) - sectors.at(index).range) < 0.1*sectors.at(index).range) )
    {
      sectors[index].angle_max += laser.angle_increment;
    }
    else
    {
      sector.angle_min = sectors.at(index).angle_max + laser.angle_increment;
      sector.angle_max = sector.angle_min;
      sector.range = laser.ranges.at(i);
      sector.range = sector.range  > laser.range_max ? laser.range_max : sector.range;
      sector.range = sector.range  < laser.range_min ? laser.range_max : sector.range;
      sectors.push_back(sector);
      index++;
    }
  }

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.a = 1.0;
  marker.ns = "my_namespace";
  marker.id = 0;

  marker.action = visualization_msgs::Marker::ADD;

  marker.header = laser.header;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

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

  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;

  /// For detecting a corridor
  for(unsigned int i=1; i< (sectors.size()-1); ++i)
  {
    if(sectors.at(i-1).range < sectors.at(i).range && sectors.at(i).range > sectors.at(i+1).range)
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
  }
  sector_pub_.publish(marker);
*/


}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "laser_processor");

  LaserProcessor lp;

  ros::spin();

  return 0;
}
