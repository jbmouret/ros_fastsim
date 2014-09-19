#define USE_SDL
#include <sstream>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "fastsim.hpp"

using namespace fastsim;

int main(int argc, char **argv) {
  ros::init(argc, argv, "fastsim");
  ROS_INFO("running fastsim\n");

  ros::NodeHandle n;
  ros::Publisher sensor_laser = 
    n.advertise<std_msgs::Float32MultiArray>("lasers", 1000);
  ros::Publisher sensor_radar = 
    n.advertise<std_msgs::Int16>("radar", 1000);
  ros::Publisher left_bumper = 
    n.advertise<std_msgs::Bool>("left_bumper", 1000);
  ros::Publisher right_bumper = 
    n.advertise<std_msgs::Bool>("right_bumper", 1000);


  /*  ros::Subscriber speed_left = 
    n.subscribe("speed_left", 1000, speed_left_cb);
  ros::Subscriber speed_right = 
    n.subscribe("speed_right", 1000, speed_right_cb);
  */

  ros::Rate loop_rate(10);
  std::string map_name;

  if (!n.getParam("map_name", map_name))
    map_name = "envs/cuisine.pbm";
  ROS_INFO("map is: %s", map_name.c_str());

  int n_lasers;
  if (!n.getParam("m_lasers", n_lasers))
    n_lasers = 20;
  ROS_INFO("n_laser is %d", n_lasers);
  
  
  boost::shared_ptr<Map> m =
    boost::shared_ptr<Map>(new Map(map_name.c_str(), 600));
  m->add_goal(Goal(100, 100, 10, 0));

  Robot r(20.0f, Posture(300, 600-60, 0));
  for (size_t i = 0; i < n_lasers; ++i)
    r.add_laser(Laser(2 * M_PI / n_lasers * i, 100.0f));

  r.add_radar(Radar(0, 16, true));

  Display d(m, r);
  
  
  while (ros::ok()) {
    // lasers
    std_msgs::Float32MultiArray laser_msg;
    for (size_t i = 0; i < n_lasers; ++i)
      laser_msg.data.push_back(r.get_lasers()[i].get_dist());
    sensor_laser.publish(laser_msg);

    // bumpers
    std_msgs::Bool msg_left_bumper, msg_right_bumper;
    msg_left_bumper.data = r.get_left_bumper();
    msg_right_bumper.data = r.get_right_bumper();
    left_bumper.publish(msg_left_bumper);
    right_bumper.publish(msg_right_bumper);

    // radar
    std_msgs::Int16 msg_radar;
    msg_radar.data = r.get_radars()[0].get_activated_slice();
    sensor_radar.publish(msg_radar);


    ros::spinOnce();

    d.update();
    r.move(1.0, 1.1, m);


  }
  
  return 0;
}
