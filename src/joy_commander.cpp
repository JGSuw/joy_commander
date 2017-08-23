// TODO license
// Author: Joseph Sullivan
// University of Washington Dept. of Biology
// July 12, 2017


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "joy_commander.h"
#include <joy_commander/button_srv.h>
#include <joy_commander/add_button_srv.h>


#define MSG_QUEUE_SIZE (uint32_t) 100


JoyCommander::JoyCommander(ros::NodeHandle nh) :
  m_nh(nh)
  {

    for(int i = 0; i < m_button_count; i++) {m_button_state[i]=0;}

    // init subscription
    std::string joy_sub_name;
    ros::param::get("joy_sub_name", joy_sub_name);
    m_joy_sub = m_nh.subscribe(joy_sub_name, MSG_QUEUE_SIZE, &JoyCommander::joy_callback, this);


    // init Publisher
    std::string twist_pub_name;
    ros::param::get("twist_pub_name", twist_pub_name);
    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>(twist_pub_name, MSG_QUEUE_SIZE);

    // get params
    this->update_parameters();

    // Setup Service server for addding button service clients (I'm so sorry you had to read that)
    std::string srv_name = "add_button_srv";
    m_add_button_srv_server = m_nh.advertiseService(srv_name, &JoyCommander::add_button_srv_cb, this);
  }


void JoyCommander::joy_callback(const sensor_msgs::Joy & msg) {

  geometry_msgs::Twist joy_command;

  if (m_control_xy) {
    joy_command.linear.x = msg.axes[m_x_axis] * m_x_sign * m_linear_x_max;
    joy_command.linear.y = msg.axes[m_y_axis] * m_y_sign * m_linear_y_max;
  } else {
    joy_command.linear.x = msg.axes[m_pitch_axis] * m_pitch_sign * m_pitch_max;
    joy_command.linear.y = msg.axes[m_roll_axis] * m_roll_sign * m_roll_max;
  }

  joy_command.linear.z = msg.axes[m_z_axis] * m_z_sign * m_z_max;
  joy_command.angular.z = msg.axes[m_yaw_axis] * m_yaw_sign * m_yaw_max;
  joy_command.angular.x = 0.;
  joy_command.angular.y = 0.;

  m_twist_pub.publish(joy_command);

  this->update_button_state((std::vector<int>)msg.buttons);

}


void JoyCommander::update_button_state(std::vector<int> buttons) {
  for(int i = 0; i < m_button_count; i++) {

    if( buttons[i] != m_button_state[i]) {
      joy_commander::button_srv service;
      service.request.button_state = buttons[i] > 0;
      std::vector<ros::ServiceClient> * client = & m_service_clients[i];

      std::vector<ros::ServiceClient>::iterator it = client->begin();

      for(;it != client->end(); it++) {
        (*it).call(service);
      }

    }
    m_button_state[i] = buttons[i];
  }
}


void JoyCommander::update_parameters() {
  ros::param::get("z_axis", m_z_axis);
  ros::param::get("z_sign", m_z_sign);
  ros::param::get("yaw_axis", m_yaw_axis);
  ros::param::get("yaw_sign", m_yaw_sign);
  ros::param::get("roll_axis", m_roll_axis);
  ros::param::get("roll_sign", m_roll_sign);
  ros::param::get("pitch_axis", m_pitch_axis);
  ros::param::get("pitch_sign", m_pitch_sign);
  ros::param::get("z_max", m_z_max);
  ros::param::get("yaw_max", m_yaw_max);
  ros::param::get("roll_max", m_roll_max);
  ros::param::get("pitch_max", m_pitch_max);
  ros::param::get("control_xy", m_control_xy);
  ros::param::get("x_axis", m_x_axis);
  ros::param::get("x_sign", m_x_sign);
  ros::param::get("y_axis", m_y_axis);
  ros::param::get("y_sign", m_y_sign);
  ros::param::get("linear_x_max", m_linear_x_max);
  ros::param::get("linear_y_max", m_linear_y_max);
}


bool JoyCommander::add_button_srv_cb(joy_commander::add_button_srv::Request & request,
                                joy_commander::add_button_srv::Response & response)
{
  m_service_clients[request.button].push_back(m_nh.serviceClient<joy_commander::button_srv>(request.service_name));
  response.str = "ok";
  return true;
}

int main(int argc, char * argv []) {

  ros::init(argc, argv, "joy_commander");
  ros::NodeHandle nh;

  JoyCommander node(nh);

  ros::Rate loop(100.);
  while(ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }
}
