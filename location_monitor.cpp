#include <vector>
#include <string>
#include <math.h>
#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandMarkDistance.h"

using location_monitor::LandMarkDistance;
using std::vector;
using std::string;

class Landmark {
  public:
    Landmark(string name, double x, double y)
      :name(name), x(x), y(y) {}
    string name;
    double x;
    double y;
};

class LandMarkMonitor {
  public:
  LandMarkMonitor(const ros::Publisher& landmark_pub): landmarks_(), landmark_pub_(landmark_pub) {
      InitLandmarks();
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    LandMarkDistance ld= FindClosest(x,y);
    ROS_INFO("name: %s, d: %f", ld.name.c_str(), ld.distance);
    landmark_pub_.publish(ld);
    if (ld.distance <=0.5){
      ROS_INFO( "I'm near the %s", ld.name.c_str());
    }
  }

private:
  vector<Landmark> landmarks_ ;
  ros::Publisher landmark_pub_;

  LandMarkDistance FindClosest(double x, double y) {
    LandMarkDistance result;
    result.distance = -1;

    for (size_t i=0; i <landmarks_.size(); ++i){
      Landmark landmark = landmarks_[i];
      double xd = landmark.x - x;
      double yd = landmark.y - y;
      double distance = sqrt(xd*xd + yd*yd);
      if(result.distance <0 || distance < result.distance){
          result.name = landmark.name;
          result.distance = distance;
      }
    }
    return result;
  }

  void InitLandmarks() {
    landmarks_.push_back(Landmark("Cube",0.31, -0.99));
    landmarks_.push_back(Landmark("Cylinder", -1.14 , -2.88));
    landmarks_.push_back(Landmark("Barrier", -2.59, -0.83));
    landmarks_.push_back(Landmark("Bookself", -0.09, 0.53));
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle nh;
  ros::Publisher landmark_pub = nh.advertise<LandMarkDistance>("closest_landmark",10);
  LandMarkMonitor monitor(landmark_pub);
  ros::Subscriber sub = nh.subscribe("odom", 10, &LandMarkMonitor::OdomCallback, &monitor);
  ros::spin();
  return 0;
}
