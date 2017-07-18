// TODO license
// Author: Joseph Sullivan
// University of Washington Dept. of Biology
// July 12, 2017

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <joy_commander/button_srv.h>
#include <joy_commander/add_button_srv.h>

class JoyCommander {

public:

  JoyCommander(ros::NodeHandle nh);

  void joy_callback(const sensor_msgs::Joy & msg);

  void update_parameters();

  bool add_button_srv_cb(joy_commander::add_button_srv::Request &
                        , joy_commander::add_button_srv::Response &);

  void update_button_state(std::vector<int> buttons);

private:
  ros::NodeHandle m_nh;

  ros::Subscriber m_joy_sub;
  ros::Publisher m_twist_pub;

  sensor_msgs::Joy m_joy_msg;

  geometry_msgs::Twist m_joy_twist;

  int m_z_axis;
  int m_z_sign;
  int m_yaw_axis;
  int m_yaw_sign;
  int m_roll_axis;
  int m_roll_sign;
  int m_pitch_axis;
  int m_pitch_sign;

  double m_z_max;
  double m_yaw_max;
  double m_roll_max;
  double m_pitch_max;

  int m_control_xy;
  int m_x_axis;
  int m_x_sign;
  int m_y_axis;
  int m_y_sign;
  double m_linear_x_max;
  double m_linear_y_max;

  static const int m_button_count = 17;
  ros::ServiceServer m_add_button_srv_server;
  std::vector<ros::ServiceClient> m_service_clients[m_button_count];
  int m_button_state[m_button_count];
};
