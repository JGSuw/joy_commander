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
    m_nh.param<std::string>("joy_sub_name", joy_sub_name, "joy");
    m_joy_sub = m_nh.subscribe(joy_sub_name, MSG_QUEUE_SIZE, &JoyCommander::joy_callback, this);


    // init Publisher
    std::string twist_pub_name;
    m_nh.param<std::string>("twist_pub_name", twist_pub_name, "joy_twist");
    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>(twist_pub_name, MSG_QUEUE_SIZE);

    // get params
    this->update_parameters();

    // Setup Service server for addding button service clients (I'm so sorry you had to read that)
    std::string srv_name = "add_button_srv";
    m_add_button_srv_server = m_nh.advertiseService(srv_name, &JoyCommander::add_button_srv_cb, this);
  }


void JoyCommander::joy_callback(const sensor_msgs::Joy & msg) {

  std::cout << "FOO!" << std::endl;
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
      service.request.button_state = !(buttons[i] == 0);

      std::vector<ros::ServiceClient> * client = & m_service_clients[i];

      std::vector<ros::ServiceClient>::iterator it = client->end();

      for(;it != client->end(); it++) {
        joy_commander::button_srv service;
        (*it).call(service);
      }

    }
    m_button_state[i] = buttons[i];
  }
}


void JoyCommander::update_parameters() {
  m_nh.param<int>("z_axis", m_z_axis, 1);
  m_nh.param<int>("z_sign", m_z_sign, 1);
  m_nh.param<int>("yaw_axis", m_yaw_axis, 0);
  m_nh.param<int>("yaw_sign", m_yaw_sign, 1);
  m_nh.param<int>("roll_axis", m_roll_axis, 2);
  m_nh.param<int>("roll_sign", m_roll_sign, -1);
  m_nh.param<int>("pitch_axis", m_pitch_axis, 3);
  m_nh.param<int>("pitch_sign", m_pitch_sign, 1);
  m_nh.param<double>("z_max", m_z_max, 0.5);
  m_nh.param<double>("yaw_max", m_yaw_max, 180.);
  m_nh.param<double>("roll_max", m_roll_max, 15.);
  m_nh.param<double>("pitch_max", m_pitch_max, 15.);
  m_nh.param<int>("control_xy", m_control_xy, 0);
  m_nh.param<int>("x_axis", m_x_axis, 3);
  m_nh.param<int>("x_sign", m_x_sign, 1);
  m_nh.param<int>("y_axis", m_y_axis, 2);
  m_nh.param<int>("y_sign", m_y_sign, 1);
  m_nh.param<double>("linear_x_max", m_linear_x_max, 1.);
  m_nh.param<double>("linear_y_max", m_linear_y_max, 1.);
}


bool JoyCommander::add_button_srv_cb(joy_commander::add_button_srv::Request & request,
                                joy_commander::add_button_srv::Response & response)
{
  m_service_clients[request.button].push_back(m_nh.serviceClient<joy_commander::button_srv>(request.service_name));
  response.str = "ok";
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
