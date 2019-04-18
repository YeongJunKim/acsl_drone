#ifndef ACSL_MISSION_H
#define ACSL_MISSION_H


// System includes
#include "unistd.h"
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/ACSL_local_position.h>
#include <dji_sdk/ACSL_local_position_delta.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

// SDK core library
#include <djiosdk/dji_vehicle.hpp>

// ROS includes

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

float target_offset_x;
float target_offset_y;
float target_offset_z;
float target_yaw;
/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;

  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;

  bool finished;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false)
  {
  }

  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

};

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

// service request
extern ServiceAck activate();

extern ServiceAck obtainCtrlAuthority();

extern ServiceAck takeoff();

extern ServiceAck land();

extern ServiceAck goHome();


// callback function
void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void twist_callback(const geometry_msgs::Twist& msg);

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);

void ACSL_local_position_callback(const dji_sdk::ACSL_local_position::ConstPtr& msg);

void ACSL_local_position_delta_callback(const dji_sdk::ACSL_local_position_delta::ConstPtr& msg);

// user function

bool drone_user_delta_control(void);

bool drone_user_control(void);

bool drone_obtainCtrlAuthority(void);

bool drone_activate(void);

bool drone_landing(void);

bool drone_takeoff(void);

bool drone_gohome(void);

bool local_set_target(float x, float y, float z, float yaw);

void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd);

bool drone_set_local_position(void);



#endif